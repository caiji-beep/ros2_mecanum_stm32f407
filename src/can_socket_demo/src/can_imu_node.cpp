#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

/**
 * @brief 基于 SocketCAN 的 IMU 数据采集节点。
 *
 * 该节点通过 Linux SocketCAN 读取 IMU 发送的 CAN 数据帧，
 * 解析加速度、角速度和姿态角，并发布为 sensor_msgs::msg::Imu 消息。
 *
 * CAN 数据帧约定：
 * - 0x180：线加速度数据；
 * - 0x181：角速度数据；
 * - 0x182：姿态角数据，并触发 IMU 消息发布。
 */
class CanImuNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数，初始化参数、CAN 接口、发布器和定时器。
   */
  CanImuNode()
  : Node("can_imu_node")
  {
    // 声明节点参数。
    declare_parameter<std::string>("interface", "can0");
    declare_parameter<std::string>("imu_topic", "imu/data_raw");
    declare_parameter<std::string>("frame_id", "");
    declare_parameter<std::string>("tf_prefix", "");

    const auto if_name = get_parameter("interface").as_string();
    const auto imu_topic = get_parameter("imu_topic").as_string();

    // 若未显式指定 frame_id，则根据 tf_prefix 生成默认 IMU 坐标系。
    frame_id_ = get_parameter("frame_id").as_string();

    if (frame_id_.empty()) {
      frame_id_ = join_frame(get_parameter("tf_prefix").as_string(), "imu_link");
    }

    // 打开 CAN 接口并创建 IMU 数据发布器。
    open_socket(if_name);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);

    // 周期性轮询 CAN 接口。2 ms 对应较高频率读取，避免缓存积压。
    timer_ = create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&CanImuNode::poll_can, this));

    RCLCPP_INFO(
      get_logger(),
      "CAN IMU listening on %s, publishing '%s' with frame_id '%s'.",
      if_name.c_str(), imu_topic.c_str(), frame_id_.c_str());
  }

  /**
   * @brief 析构函数，关闭 CAN socket。
   */
  ~CanImuNode() override
  {
    if (sock_ >= 0) {
      close(sock_);
    }
  }

private:
  /**
   * @brief 清理 TF 前缀中的首尾斜杠。
   *
   * @param prefix 原始 TF 前缀。
   * @return 清理后的 TF 前缀。
   */
  static std::string clean_prefix(std::string prefix)
  {
    while (!prefix.empty() && prefix.front() == '/') {
      prefix.erase(prefix.begin());
    }

    while (!prefix.empty() && prefix.back() == '/') {
      prefix.pop_back();
    }

    return prefix;
  }

  /**
   * @brief 拼接 TF 前缀和坐标系名称。
   *
   * @param prefix TF 前缀。
   * @param frame 坐标系名称。
   * @return 完整 frame 名称。
   */
  static std::string join_frame(const std::string & prefix, const std::string & frame)
  {
    const auto clean = clean_prefix(prefix);

    return clean.empty() ? frame : clean + "/" + frame;
  }

  /**
   * @brief 将两个字节转换为 int16_t 有符号整数。
   *
   * IMU CAN 数据采用低字节在前、高字节在后的格式。
   *
   * @param lo 低字节。
   * @param hi 高字节。
   * @return 转换后的 int16_t 数值。
   */
  static int16_t bytes_to_int16(uint8_t lo, uint8_t hi)
  {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
  }

  /**
   * @brief 打开并配置 SocketCAN 接口。
   *
   * @param if_name CAN 网络接口名称，例如 can0。
   *
   * @throw std::runtime_error 当 socket、ioctl、bind 或 fcntl 配置失败时抛出异常。
   */
  void open_socket(const std::string & if_name)
  {
    // 创建原始 CAN socket。
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (sock_ < 0) {
      RCLCPP_FATAL(get_logger(), "socket() failed: %s", std::strerror(errno));
      throw std::runtime_error("socket() failed");
    }

    // 根据接口名称查询 CAN 接口索引。
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, if_name.c_str(), IFNAMSIZ - 1);

    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_FATAL(
        get_logger(),
        "ioctl(SIOCGIFINDEX) failed for %s: %s",
        if_name.c_str(), std::strerror(errno));

      close(sock_);
      sock_ = -1;
      throw std::runtime_error("ioctl(SIOCGIFINDEX) failed");
    }

    // 绑定 socket 到指定 CAN 接口。
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_FATAL(get_logger(), "bind() failed: %s", std::strerror(errno));

      close(sock_);
      sock_ = -1;
      throw std::runtime_error("bind() failed");
    }

    // 设置为非阻塞模式，避免定时器回调被 read 长时间阻塞。
    const int flags = fcntl(sock_, F_GETFL, 0);

    if (flags < 0 || fcntl(sock_, F_SETFL, flags | O_NONBLOCK) < 0) {
      RCLCPP_FATAL(get_logger(), "fcntl(O_NONBLOCK) failed: %s", std::strerror(errno));

      close(sock_);
      sock_ = -1;
      throw std::runtime_error("fcntl(O_NONBLOCK) failed");
    }
  }

  /**
   * @brief 轮询 CAN 接口并分发 IMU 数据帧。
   *
   * 该函数由定时器周期调用。根据 CAN ID 区分加速度、角速度和姿态角数据。
   */
  void poll_can()
  {
    struct can_frame frame;
    const auto nbytes = read(sock_, &frame, sizeof(frame));

    // 非阻塞读取下，暂无数据属于正常情况。
    if (nbytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }

      RCLCPP_WARN(get_logger(), "read() error: %s", std::strerror(errno));
      return;
    }

    if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
      RCLCPP_WARN(get_logger(), "incomplete CAN frame");
      return;
    }

    // 仅保留标准帧 ID。
    const uint32_t id = frame.can_id & CAN_SFF_MASK;

    // 当前协议至少需要 6 字节数据和 1 字节序号。
    if (frame.can_dlc < 7) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "CAN frame 0x%03X ignored because dlc=%u is too short.",
        id, frame.can_dlc);

      return;
    }

    // 按 CAN ID 更新对应 IMU 数据。
    if (id == 0x180) {
      update_acceleration(frame);
    } else if (id == 0x181) {
      update_gyro(frame);
    } else if (id == 0x182) {
      update_orientation(frame);

      // 姿态帧到达后，认为一组 IMU 数据基本完整，发布消息。
      publish_imu();
    }
  }

  /**
   * @brief 解析线加速度数据帧。
   *
   * 数据比例：raw / 1000.0 表示 g，再乘以重力加速度转换为 m/s^2。
   */
  void update_acceleration(const struct can_frame & frame)
  {
    const auto ax_raw = bytes_to_int16(frame.data[0], frame.data[1]);
    const auto ay_raw = bytes_to_int16(frame.data[2], frame.data[3]);
    const auto az_raw = bytes_to_int16(frame.data[4], frame.data[5]);

    constexpr double gravity = 9.81;

    last_ax_ms2_ = (ax_raw / 1000.0) * gravity;
    last_ay_ms2_ = (ay_raw / 1000.0) * gravity;
    last_az_ms2_ = (az_raw / 1000.0) * gravity;
    last_seq_ = frame.data[6];
  }

  /**
   * @brief 解析角速度数据帧。
   *
   * 数据比例：raw / 1000.0。
   * 具体单位应与 IMU 设备协议保持一致，通常应为 rad/s 或 deg/s。
   */
  void update_gyro(const struct can_frame & frame)
  {
    last_gx_ = bytes_to_int16(frame.data[0], frame.data[1]) / 1000.0;
    last_gy_ = bytes_to_int16(frame.data[2], frame.data[3]) / 1000.0;
    last_gz_ = bytes_to_int16(frame.data[4], frame.data[5]) / 1000.0;
    last_seq_ = frame.data[6];
  }

  /**
   * @brief 解析姿态角数据帧。
   *
   * 数据比例：raw / 1000.0。
   * 当前代码默认解析结果可直接作为 roll、pitch、yaw 输入四元数转换。
   */
  void update_orientation(const struct can_frame & frame)
  {
    last_roll_ = bytes_to_int16(frame.data[0], frame.data[1]) / 1000.0;
    last_pitch_ = bytes_to_int16(frame.data[2], frame.data[3]) / 1000.0;
    last_yaw_ = bytes_to_int16(frame.data[4], frame.data[5]) / 1000.0;
    last_seq_ = frame.data[6];
  }

  /**
   * @brief 发布 ROS 2 IMU 消息。
   *
   * 将最近一次解析到的姿态、角速度和线加速度填充到
   * sensor_msgs::msg::Imu 消息中并发布。
   */
  void publish_imu()
  {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    // 将 RPY 姿态角转换为四元数。
    tf2::Quaternion q;
    q.setRPY(last_roll_, last_pitch_, last_yaw_);

    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();

    msg.angular_velocity.x = last_gx_;
    msg.angular_velocity.y = last_gy_;
    msg.angular_velocity.z = last_gz_;

    msg.linear_acceleration.x = last_ax_ms2_;
    msg.linear_acceleration.y = last_ay_ms2_;
    msg.linear_acceleration.z = last_az_ms2_;

    // 初始化协方差矩阵。
    for (int i = 0; i < 9; ++i) {
      msg.orientation_covariance[i] = 0.0;
      msg.angular_velocity_covariance[i] = 0.0;
      msg.linear_acceleration_covariance[i] = 0.0;
    }

    // 设置对角线协方差估计值。
    msg.orientation_covariance[0] = 0.05;
    msg.orientation_covariance[4] = 0.05;
    msg.orientation_covariance[8] = 0.1;

    msg.angular_velocity_covariance[0] = 0.01;
    msg.angular_velocity_covariance[4] = 0.01;
    msg.angular_velocity_covariance[8] = 0.02;

    msg.linear_acceleration_covariance[0] = 0.1;
    msg.linear_acceleration_covariance[4] = 0.1;
    msg.linear_acceleration_covariance[8] = 0.2;

    imu_pub_->publish(msg);
  }

  // CAN socket 文件描述符。
  int sock_{-1};

  // CAN 轮询定时器。
  rclcpp::TimerBase::SharedPtr timer_;

  // IMU 消息发布器。
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // IMU 消息使用的坐标系名称。
  std::string frame_id_;

  // 最近一次解析得到的线加速度，单位 m/s^2。
  double last_ax_ms2_{0.0};
  double last_ay_ms2_{0.0};
  double last_az_ms2_{0.0};

  // 最近一次解析得到的角速度。
  double last_gx_{0.0};
  double last_gy_{0.0};
  double last_gz_{0.0};

  // 最近一次解析得到的姿态角。
  double last_roll_{0.0};
  double last_pitch_{0.0};
  double last_yaw_{0.0};

  // 最近一次接收到的数据帧序号。
  uint8_t last_seq_{0};
};

/**
 * @brief 程序入口函数。
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CanImuNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}