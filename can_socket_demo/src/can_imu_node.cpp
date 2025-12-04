#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

extern "C" {
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <errno.h>
}

class CanImuNode : public rclcpp::Node
{
public:
    CanImuNode()
    : Node("can_imu_node")
    {
        // 1. 参数：接口名，默认 can0
        this->declare_parameter<std::string>("interface", "can0");
        std::string if_name = this->get_parameter("interface").as_string();

        // 2. 打开 SocketCAN
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) {
            RCLCPP_FATAL(get_logger(), "socket() failed");
            throw std::runtime_error("socket() failed");
        }

        struct ifreq ifr;
        std::memset(&ifr, 0, sizeof(ifr));
        std::strncpy(ifr.ifr_name, if_name.c_str(), IFNAMSIZ - 1);

        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_FATAL(get_logger(), "ioctl(SIOCGIFINDEX) failed for %s", if_name.c_str());
            close(sock_);
            throw std::runtime_error("ioctl failed");
        }

        std::memset(&addr_, 0, sizeof(addr_));
        addr_.can_family  = AF_CAN;
        addr_.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0) {
            RCLCPP_FATAL(get_logger(), "bind() failed");
            close(sock_);
            throw std::runtime_error("bind failed");
        }

        // 非阻塞
        int flags = fcntl(sock_, F_GETFL, 0);
        fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

        RCLCPP_INFO(get_logger(), "CAN socket bound to interface: %s", if_name.c_str());

        // 3. 发布器：IMU 数据
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("robmini/imu/data_raw", 10);

        // 4. 定时器轮询 CAN
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&CanImuNode::poll_can, this));
    }

    ~CanImuNode() override
    {
        if (sock_ >= 0) {
            close(sock_);
        }
    }

private:
    // 小工具：从两个字节还原 int16_t（小端）
    static int16_t bytes_to_int16(uint8_t lo, uint8_t hi)
    {
        return (int16_t)((hi << 8) | lo);
    }

    void poll_can()
    {
        struct can_frame frame;
        int nbytes = read(sock_, &frame, sizeof(frame));

        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return; // 没数据
            } else {
                RCLCPP_WARN(get_logger(), "read() error: %s", strerror(errno));
                return;
            }
        }

        if (nbytes < (int)sizeof(struct can_frame)) {
            RCLCPP_WARN(get_logger(), "incomplete CAN frame");
            return;
        }

        uint32_t id  = frame.can_id & CAN_SFF_MASK;  // 标准帧 ID

        if (id == 0x180 && frame.can_dlc >= 6) {
            // 加速度 ax, ay, az （单位：g，发送时 *1000）
            int16_t ax_raw = bytes_to_int16(frame.data[0], frame.data[1]);
            int16_t ay_raw = bytes_to_int16(frame.data[2], frame.data[3]);
            int16_t az_raw = bytes_to_int16(frame.data[4], frame.data[5]);
            uint8_t seq    = frame.data[6];

            double ax_g = ax_raw / 1000.0;  // g
            double ay_g = ay_raw / 1000.0;
            double az_g = az_raw / 1000.0;

            // 转成 m/s^2
            last_ax_ms2_ = ax_g * 9.81;
            last_ay_ms2_ = ay_g * 9.81;
            last_az_ms2_ = az_g * 9.81;
            last_seq_    = seq;

            RCLCPP_DEBUG(get_logger(),
                         "ACC seq=%u ax=%.3f g ay=%.3f g az=%.3f g",
                         seq, ax_g, ay_g, az_g);
        }
        else if (id == 0x181 && frame.can_dlc >= 6) {
            // 角速度 gx,gy,gz（单位：rad/s，发送时 *1000）
            int16_t gx_raw = bytes_to_int16(frame.data[0], frame.data[1]);
            int16_t gy_raw = bytes_to_int16(frame.data[2], frame.data[3]);
            int16_t gz_raw = bytes_to_int16(frame.data[4], frame.data[5]);
            uint8_t seq    = frame.data[6];

            double gx = gx_raw / 1000.0;   // rad/s
            double gy = gy_raw / 1000.0;
            double gz = gz_raw / 1000.0;

            last_gx_ = gx;
            last_gy_ = gy;
            last_gz_ = gz;
            last_seq_ = seq;

            RCLCPP_DEBUG(get_logger(),
                         "GYR seq=%u gx=%.3f rad/s gy=%.3f gz=%.3f",
                         seq, gx, gy, gz);
        }
        else if (id == 0x182 && frame.can_dlc >= 6) {
            // 姿态角 roll,pitch,yaw（单位：rad，发送时 *1000）
            int16_t roll_raw  = bytes_to_int16(frame.data[0], frame.data[1]);
            int16_t pitch_raw = bytes_to_int16(frame.data[2], frame.data[3]);
            int16_t yaw_raw   = bytes_to_int16(frame.data[4], frame.data[5]);
            uint8_t seq       = frame.data[6];

            double roll  = roll_raw  / 1000.0;   // rad
            double pitch = pitch_raw / 1000.0;
            double yaw   = yaw_raw   / 1000.0;

            last_roll_  = roll;
            last_pitch_ = pitch;
            last_yaw_   = yaw;
            last_seq_   = seq;

            RCLCPP_DEBUG(get_logger(),
                         "RPY seq=%u roll=%.3f rad pitch=%.3f yaw=%.3f",
                         seq, roll, pitch, yaw);

            // 约定：收到 0x182 时，用当前保存的 acc+gyro+姿态 发一条完整的 IMU
            publish_imu();
        }
        else {
            // 有其他 ID，先不处理
        }
    }

    void publish_imu()
    {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "robmini/imu_link";  

        // 1) orientation：由 roll,pitch,yaw 转 quaternion
        tf2::Quaternion q;
        q.setRPY(last_roll_, last_pitch_, last_yaw_);
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();

        // 2) angular velocity：rad/s
        msg.angular_velocity.x = last_gx_;
        msg.angular_velocity.y = last_gy_;
        msg.angular_velocity.z = last_gz_;

        // 3) linear acceleration：m/s^2
        msg.linear_acceleration.x = last_ax_ms2_;
        msg.linear_acceleration.y = last_ay_ms2_;
        msg.linear_acceleration.z = last_az_ms2_;

        // 简单给一点对角协方差（你后面可以根据测量噪声调整）
        for (int i = 0; i < 9; ++i) {
            msg.orientation_covariance[i]        = 0.0;
            msg.angular_velocity_covariance[i]   = 0.0;
            msg.linear_acceleration_covariance[i]= 0.0;
        }
        msg.orientation_covariance[0]      = 0.05;
        msg.orientation_covariance[4]      = 0.05;
        msg.orientation_covariance[8]      = 0.1;

        msg.angular_velocity_covariance[0] = 0.01;
        msg.angular_velocity_covariance[4] = 0.01;
        msg.angular_velocity_covariance[8] = 0.02;

        msg.linear_acceleration_covariance[0] = 0.1;
        msg.linear_acceleration_covariance[4] = 0.1;
        msg.linear_acceleration_covariance[8] = 0.2;

        imu_pub_->publish(msg);

        RCLCPP_INFO(get_logger(),
                    "IMU pub seq=%u: acc[%.2f %.2f %.2f] m/s^2, "
                    "gyro[%.3f %.3f %.3f] rad/s, rpy[%.3f %.3f %.3f] rad",
                    last_seq_,
                    last_ax_ms2_, last_ay_ms2_, last_az_ms2_,
                    last_gx_, last_gy_, last_gz_,
                    last_roll_, last_pitch_, last_yaw_);
    }

    int sock_{-1};
    struct sockaddr_can addr_{};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // 最近一次解析得到的数据
    double last_ax_ms2_{0.0}, last_ay_ms2_{0.0}, last_az_ms2_{0.0};
    double last_gx_{0.0}, last_gy_{0.0}, last_gz_{0.0};
    double last_roll_{0.0}, last_pitch_{0.0}, last_yaw_{0.0};
    uint8_t last_seq_{0};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
