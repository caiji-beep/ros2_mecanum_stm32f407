#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 麦克纳姆轮键盘遥控节点
 *
 * 该节点通过终端键盘读取控制指令，并发布 geometry_msgs::msg::Twist
 * 到指定 cmd_vel 话题，用于控制麦克纳姆轮底盘运动。
 *
 * 控制按键：
 * - w / s：前进 / 后退
 * - a / d：左平移 / 右平移
 * - q / e：左旋 / 右旋
 * - x：停止
 * - Ctrl+C：退出程序
 */
class MecanumTeleopKeyboard : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数，初始化参数、发布器和键盘读取线程。
   */
  MecanumTeleopKeyboard()
  : Node("mecanum_teleop_keyboard")
  {
    // 声明键盘控制参数。
    declare_parameter<double>("linear_speed", 0.2);
    declare_parameter<double>("angular_speed", 1.0);
    declare_parameter<double>("strafe_speed", 0.2);
    declare_parameter<double>("command_timeout", 0.2);
    declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

    // 读取参数值。
    linear_speed_ = get_parameter("linear_speed").as_double();
    angular_speed_ = get_parameter("angular_speed").as_double();
    strafe_speed_ = get_parameter("strafe_speed").as_double();
    command_timeout_ = get_parameter("command_timeout").as_double();
    const auto cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();

    // 创建速度指令发布器。
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    input_fd_ = STDIN_FILENO;
    if (!isatty(input_fd_)) {
      const int tty_fd = ::open("/dev/tty", O_RDONLY);
      if (tty_fd >= 0) {
        input_fd_ = tty_fd;
        owns_input_fd_ = true;
      }
    }

    // 保存终端原始配置，后续读取键盘时会临时切换为非规范模式。
    terminal_available_ = tcgetattr(input_fd_, &original_termios_) == 0;
    if (!terminal_available_) {
      RCLCPP_WARN(
        get_logger(),
        "No interactive terminal is available. Run this node in a TTY if keyboard input is needed.");
    }

    last_cmd_time_ = now();

    // 启动键盘监听线程，避免阻塞 ROS 2 spin。
    keyboard_thread_ = std::thread(&MecanumTeleopKeyboard::keyboard_loop, this);

    RCLCPP_INFO(get_logger(), "Mecanum teleop started, publishing to '%s'.", cmd_vel_topic.c_str());
    RCLCPP_INFO(get_logger(), "Keys: w/s forward/back, a/d strafe, q/e rotate, x stop, Ctrl+C quit.");
  }

  /**
   * @brief 析构函数，停止线程、恢复终端并发布停止指令。
   */
  ~MecanumTeleopKeyboard() override
  {
    running_.store(false);

    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }

    publish_stop();
    restore_terminal();

    if (owns_input_fd_ && input_fd_ >= 0) {
      ::close(input_fd_);
    }
  }

private:
  /**
   * @brief 非阻塞读取单个键盘字符。
   *
   * 函数会临时将终端设置为非规范模式，使程序可以不按回车直接读取按键。
   *
   * @return 读取到的字符；若超时未读取到按键，则返回 0。
   */
  char read_key()
  {
    if (!terminal_available_) {
      return 0;
    }

    termios raw = original_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(input_fd_, TCSANOW, &raw);

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(input_fd_, &readfds);

    // 设置 select 超时时间为 50 ms，避免线程长时间阻塞。
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 50000;

    char key = 0;

    if (select(input_fd_ + 1, &readfds, nullptr, nullptr, &timeout) > 0) {
      (void)read(input_fd_, &key, 1);
    }

    restore_terminal();
    return key;
  }

  /**
   * @brief 恢复终端原始配置。
   */
  void restore_terminal()
  {
    if (terminal_available_) {
      tcsetattr(input_fd_, TCSANOW, &original_termios_);
    }
  }

  /**
   * @brief 发布零速度指令，使机器人停止运动。
   */
  void publish_stop()
  {
    geometry_msgs::msg::Twist twist;
    cmd_vel_publisher_->publish(twist);
  }

  /**
   * @brief 键盘监听主循环。
   *
   * 根据按键生成速度指令；若长时间没有有效按键，则自动发布停止指令，
   * 防止机器人持续执行上一条速度命令。
   */
  void keyboard_loop()
  {
    using namespace std::chrono_literals;

    while (rclcpp::ok() && running_.load()) {
      auto requested_twist = geometry_msgs::msg::Twist();
      const char key = read_key();
      bool handled = false;

      // 根据键盘输入设置线速度、横向速度或角速度。
      switch (key) {
        case 'w':
          requested_twist.linear.x = linear_speed_;
          handled = true;
          break;

        case 's':
          requested_twist.linear.x = -linear_speed_;
          handled = true;
          break;

        case 'a':
          requested_twist.linear.y = strafe_speed_;
          handled = true;
          break;

        case 'd':
          requested_twist.linear.y = -strafe_speed_;
          handled = true;
          break;

        case 'q':
          requested_twist.angular.z = angular_speed_;
          handled = true;
          break;

        case 'e':
          requested_twist.angular.z = -angular_speed_;
          handled = true;
          break;

        case 'x':
          // x 对应零速度，用于主动停车。
          handled = true;
          break;

        case 3:
          // Ctrl+C 的 ASCII 码为 3。
          publish_stop();
          rclcpp::shutdown();
          return;

        default:
          break;
      }

      if (handled) {
        // 有效按键：更新时间戳并缓存当前速度指令。
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_cmd_time_ = now();
        current_twist_ = requested_twist;
      }

      geometry_msgs::msg::Twist output_twist;
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        if ((now() - last_cmd_time_).seconds() <= command_timeout_) {
          output_twist = current_twist_;
        } else {
          current_twist_ = geometry_msgs::msg::Twist();
        }
      }
      cmd_vel_publisher_->publish(output_twist);

      // 控制循环频率，减少 CPU 占用。
      std::this_thread::sleep_for(20ms);
    }
  }

  // 速度指令发布器。
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  // 速度参数。
  double linear_speed_{0.2};
  double angular_speed_{1.0};
  double strafe_speed_{0.2};

  // 指令超时时间，超过该时间未收到有效按键则发布停止指令。
  double command_timeout_{0.2};

  // 终端原始配置，用于程序退出或读取结束后恢复终端状态。
  termios original_termios_{};
  int input_fd_{STDIN_FILENO};
  bool owns_input_fd_{false};
  bool terminal_available_{false};

  // 键盘监听线程。
  std::thread keyboard_thread_;

  // 线程运行标志。
  std::atomic_bool running_{true};

  // 上一次有效速度指令的时间。
  rclcpp::Time last_cmd_time_;
  geometry_msgs::msg::Twist current_twist_;

  // 速度发布和时间更新互斥锁。
  std::mutex cmd_mutex_;
};

/**
 * @brief 程序入口函数。
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MecanumTeleopKeyboard>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
