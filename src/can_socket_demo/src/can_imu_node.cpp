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
#include <algorithm>
#include <cctype>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

/**
 * @brief 基于 SocketCAN 的 IMU 数据采集节点。
 *
 * 该节点通过 Linux SocketCAN 读取 IMU 发送的 CAN 数据帧，
 * 解析加速度、角速度和姿态角，并发布为 sensor_msgs::msg::Imu 消息。
 * 默认不发布姿态四元数，只发布角速度和线加速度，避免把无磁力计约束的 yaw 漂移当成绝对姿态融合。
 *
 * CAN 数据帧约定：
 * - 0x180：线加速度数据，ax/ay/az = g * 1000；
 * - 0x181：角速度数据，gx/gy/gz = rad/s * 1000；
 * - 0x182：姿态角数据，roll/pitch/yaw = rad * 1000，并触发 IMU 消息发布；
 * - 0x184：状态帧，包含 seq、状态 flags、错误码和恢复计数等。
 *
 * 0x180~0x182 的第 6 字节为 seq，第 7 字节为状态 flags。
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
    declare_parameter<bool>("publish_orientation", false);
    declare_parameter<std::string>("gyro_unit", "rad_per_s");
    declare_parameter<std::string>("orientation_unit", "rad");
    declare_parameter<bool>("require_calibrated", false);
    declare_parameter<double>("publish_rate_hz", 50.0);

    const auto if_name = get_parameter("interface").as_string();
    const auto imu_topic = get_parameter("imu_topic").as_string();
    publish_orientation_ = get_parameter("publish_orientation").as_bool();
    require_calibrated_ = get_parameter("require_calibrated").as_bool();
    gyro_scale_to_rad_ = unit_scale_to_rad(get_parameter("gyro_unit").as_string(), "gyro_unit");
    orientation_scale_to_rad_ =
      unit_scale_to_rad(get_parameter("orientation_unit").as_string(), "orientation_unit");
    const auto publish_rate_hz = get_parameter("publish_rate_hz").as_double();
    if (publish_rate_hz > 0.0) {
      publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_hz);
    }

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
      "CAN IMU listening on %s, publishing '%s' with frame_id '%s', publish_orientation=%s, max_rate=%.1fHz.",
      if_name.c_str(), imu_topic.c_str(), frame_id_.c_str(),
      publish_orientation_ ? "true" : "false", publish_rate_hz);
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
  static constexpr uint8_t imu_can_flag_data_valid_ = 0x01U;
  static constexpr uint8_t imu_can_flag_calibrated_ = 0x02U;
  static constexpr uint8_t imu_can_flag_recovering_ = 0x04U;
  static constexpr uint8_t imu_can_flag_fault_ = 0x08U;

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

  static uint16_t bytes_to_uint16(uint8_t lo, uint8_t hi)
  {
    return static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) | lo);
  }

  /**
   * @brief 将角度或角速度单位转换为 ROS 约定的弧度单位。
   */
  double unit_scale_to_rad(std::string unit, const std::string & parameter_name)
  {
    std::transform(unit.begin(), unit.end(), unit.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });

    unit.erase(std::remove_if(unit.begin(), unit.end(), [](unsigned char c) {
      return std::isspace(c) != 0;
    }), unit.end());

    if (
      unit == "rad" || unit == "radian" || unit == "radians" ||
      unit == "rad/s" || unit == "rad_per_s" || unit == "radps" ||
      unit == "radian_per_s" || unit == "radians_per_second")
    {
      return 1.0;
    }

    if (
      unit == "deg" || unit == "degree" || unit == "degrees" ||
      unit == "deg/s" || unit == "deg_per_s" || unit == "dps" ||
      unit == "degree_per_s" || unit == "degrees_per_second")
    {
      constexpr double pi = 3.14159265358979323846;
      return pi / 180.0;
    }

    RCLCPP_WARN(
      get_logger(),
      "Unknown %s '%s', using radians scale.",
      parameter_name.c_str(), unit.c_str());
    return 1.0;
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
    while (true) {
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

      handle_can_frame(frame);
    }
  }

  void handle_can_frame(const struct can_frame & frame)
  {
    // 仅保留标准帧 ID。
    const uint32_t id = frame.can_id & CAN_SFF_MASK;

    if (id != 0x180 && id != 0x181 && id != 0x182 && id != 0x184) {
      return;
    }

    // 下位机 IMU 协议固定使用 8 字节标准数据帧。
    if (frame.can_dlc < 8) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "CAN frame 0x%03X ignored because dlc=%u is too short.",
        id, frame.can_dlc);

      return;
    }

    if (id == 0x184) {
      update_status(frame);
      return;
    }

    const uint8_t seq = frame.data[6];
    const uint8_t status_flags = frame.data[7];
    if (!sensor_status_ok(status_flags, id)) {
      return;
    }

    // 按 CAN ID 更新对应 IMU 数据。
    if (id == 0x180) {
      update_acceleration(frame, seq);
    } else if (id == 0x181) {
      update_gyro(frame, seq);
    } else if (id == 0x182) {
      update_orientation(frame, seq);

      if (have_accel_ && have_gyro_ && last_accel_seq_ == seq && last_gyro_seq_ == seq) {
        publish_imu(seq, status_flags);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "IMU seq mismatch, waiting for a complete sample: accel=%u gyro=%u orientation=%u.",
          last_accel_seq_, last_gyro_seq_, seq);
      }
    }
  }

  bool sensor_status_ok(uint8_t flags, uint32_t id)
  {
    last_status_flags_ = flags;

    if ((flags & imu_can_flag_data_valid_) == 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU CAN frame 0x%03X ignored because data_valid is false. flags=0x%02X",
        id, flags);
      return false;
    }

    if ((flags & imu_can_flag_fault_) != 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU CAN frame 0x%03X ignored because fault flag is set. flags=0x%02X last_error=%u",
        id, flags, last_error_);
      return false;
    }

    if (require_calibrated_ && (flags & imu_can_flag_calibrated_) == 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU CAN frame 0x%03X ignored because calibrated flag is false. flags=0x%02X",
        id, flags);
      return false;
    }

    if ((flags & imu_can_flag_recovering_) != 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "IMU is recovering, using valid data with caution. flags=0x%02X",
        flags);
    }

    return true;
  }

  void update_status(const struct can_frame & frame)
  {
    last_status_seq_ = frame.data[0];
    last_status_flags_ = frame.data[1];
    last_error_ = frame.data[2];
    last_consecutive_failures_ = frame.data[3];
    last_read_fail_count_lsb_ = frame.data[4];
    last_recovery_count_lsb_ = frame.data[5];
    last_update_ms_lsb_ = bytes_to_uint16(frame.data[6], frame.data[7]);

    if ((last_status_flags_ & imu_can_flag_fault_) != 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU status fault: seq=%u flags=0x%02X error=%u failures=%u read_fail_lsb=%u recovery_lsb=%u update_ms_lsb=%u",
        last_status_seq_, last_status_flags_, last_error_, last_consecutive_failures_,
        last_read_fail_count_lsb_, last_recovery_count_lsb_, last_update_ms_lsb_);
      return;
    }

    if ((last_status_flags_ & imu_can_flag_data_valid_) == 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU status invalid: seq=%u flags=0x%02X error=%u failures=%u",
        last_status_seq_, last_status_flags_, last_error_, last_consecutive_failures_);
      return;
    }

    if ((last_status_flags_ & imu_can_flag_calibrated_) == 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "IMU status not calibrated yet: seq=%u flags=0x%02X",
        last_status_seq_, last_status_flags_);
    }
  }

  /**
   * @brief 解析线加速度数据帧。
   *
   * 数据比例：raw / 1000.0 表示 g，再乘以重力加速度转换为 m/s^2。
   */
  void update_acceleration(const struct can_frame & frame, uint8_t seq)
  {
    const auto ax_raw = bytes_to_int16(frame.data[0], frame.data[1]);
    const auto ay_raw = bytes_to_int16(frame.data[2], frame.data[3]);
    const auto az_raw = bytes_to_int16(frame.data[4], frame.data[5]);

    constexpr double gravity = 9.81;

    last_ax_ms2_ = (ax_raw / 1000.0) * gravity;
    last_ay_ms2_ = (ay_raw / 1000.0) * gravity;
    last_az_ms2_ = (az_raw / 1000.0) * gravity;
    last_accel_seq_ = seq;
    have_accel_ = true;
  }

  /**
   * @brief 解析角速度数据帧。
   *
   * 数据比例：raw / 1000.0。
   * 具体单位应与 IMU 设备协议保持一致，通常应为 rad/s 或 deg/s。
   */
  void update_gyro(const struct can_frame & frame, uint8_t seq)
  {
    last_gx_ = (bytes_to_int16(frame.data[0], frame.data[1]) / 1000.0) * gyro_scale_to_rad_;
    last_gy_ = (bytes_to_int16(frame.data[2], frame.data[3]) / 1000.0) * gyro_scale_to_rad_;
    last_gz_ = (bytes_to_int16(frame.data[4], frame.data[5]) / 1000.0) * gyro_scale_to_rad_;
    last_gyro_seq_ = seq;
    have_gyro_ = true;
  }

  /**
   * @brief 解析姿态角数据帧。
   *
   * 数据比例：raw / 1000.0。
   * 当前代码默认解析结果可直接作为 roll、pitch、yaw 输入四元数转换。
   */
  void update_orientation(const struct can_frame & frame, uint8_t seq)
  {
    last_roll_ = (bytes_to_int16(frame.data[0], frame.data[1]) / 1000.0) * orientation_scale_to_rad_;
    last_pitch_ = (bytes_to_int16(frame.data[2], frame.data[3]) / 1000.0) * orientation_scale_to_rad_;
    last_yaw_ = (bytes_to_int16(frame.data[4], frame.data[5]) / 1000.0) * orientation_scale_to_rad_;
    last_orientation_seq_ = seq;
  }

  /**
   * @brief 发布 ROS 2 IMU 消息。
   *
   * 将最近一次解析到的姿态、角速度和线加速度填充到
   * sensor_msgs::msg::Imu 消息中并发布。
   */
  void publish_imu(uint8_t seq, uint8_t status_flags)
  {
    const auto stamp = now();
    if (
      publish_period_.nanoseconds() > 0 &&
      last_publish_time_.nanoseconds() > 0 &&
      stamp - last_publish_time_ < publish_period_)
    {
      return;
    }

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    last_publish_time_ = stamp;
    last_seq_ = seq;
    last_status_flags_ = status_flags;

    if (publish_orientation_) {
      // 将 RPY 姿态角转换为四元数。无磁力计时 yaw 会漂，默认不要用于定位融合。
      tf2::Quaternion q;
      q.setRPY(last_roll_, last_pitch_, last_yaw_);

      msg.orientation.x = q.x();
      msg.orientation.y = q.y();
      msg.orientation.z = q.z();
      msg.orientation.w = q.w();
    } else {
      msg.orientation.w = 1.0;
    }

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

    if (publish_orientation_) {
      // 设置对角线协方差估计值。
      msg.orientation_covariance[0] = 0.05;
      msg.orientation_covariance[4] = 0.05;
      msg.orientation_covariance[8] = 0.1;
    } else {
      // sensor_msgs/Imu 约定：orientation_covariance[0] = -1 表示没有姿态估计。
      msg.orientation_covariance[0] = -1.0;
    }

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

  // IMU 发布限频，避免高频 CAN 数据在低性能主机上压垮 EKF 和 DDS。
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

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

  // 是否将下位机欧拉角发布为 orientation。默认关闭，避免 yaw 漂移被当成绝对航向。
  bool publish_orientation_{false};

  // 是否要求下位机标记 IMU 已完成校准后才发布数据。
  bool require_calibrated_{false};

  // 将下位机单位转换为 ROS IMU 消息使用的 rad/s 和 rad。
  double gyro_scale_to_rad_{1.0};
  double orientation_scale_to_rad_{1.0};

  // 最近一次接收到的数据帧序号。
  uint8_t last_seq_{0};
  uint8_t last_accel_seq_{0};
  uint8_t last_gyro_seq_{0};
  uint8_t last_orientation_seq_{0};
  bool have_accel_{false};
  bool have_gyro_{false};

  // 最近一次接收到的状态帧。
  uint8_t last_status_seq_{0};
  uint8_t last_status_flags_{0};
  uint8_t last_error_{0};
  uint8_t last_consecutive_failures_{0};
  uint8_t last_read_fail_count_lsb_{0};
  uint8_t last_recovery_count_lsb_{0};
  uint16_t last_update_ms_lsb_{0};
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
