/*
终端控制原理：Linux 终端默认是「规范模式」（需按回车才提交输入）和「回显模式」（输入字符显示在屏幕），代码需为「非规范模式 + 无回显」以实现「按键即响应」。
*/
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>    // C++时间库（用于线程睡眠）
#include <thread>    // C++线程库（创建键盘监听线程）
#include <termios.h>  // Linux终端控制库（修改终端模式）
#include <unistd.h>   // Linux系统调用（read、STDIN_FILENO）
#include <sys/select.h>  // Linux I/O多路复用（非阻塞等待键盘输入）

class MecanumTeleopKeyboard : public rclcpp::Node
{
public:
    MecanumTeleopKeyboard() : Node("mecanum_teleop_keyboard")
    {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/robmini/cmd_vel", 
            10);  // 消息队列大小：缓存10条消息，避免发布过快导致丢失
        
        // 参数配置
        // 语法：this->declare_parameter<参数类型>("参数名", 默认值)
        this->declare_parameter<double>("linear_speed", 0.2);   //m/s
        this->declare_parameter<double>("angular_speed", 1.0);  //rad/s
        this->declare_parameter<double>("strafe_speed", 0.2);   //m/s
        
        // 语法：this->get_parameter("参数名").as_类型() → 获取参数值
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();
        strafe_speed_ = this->get_parameter("strafe_speed").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Mecanum键盘控制节点已启动");
        RCLCPP_INFO(this->get_logger(), "控制说明:");
        RCLCPP_INFO(this->get_logger(), "  前进: w");
        RCLCPP_INFO(this->get_logger(), "  后退: s");
        RCLCPP_INFO(this->get_logger(), "  左移: a");
        RCLCPP_INFO(this->get_logger(), "  右移: d");
        RCLCPP_INFO(this->get_logger(), "  左转: q");
        RCLCPP_INFO(this->get_logger(), "  右转: e");
        RCLCPP_INFO(this->get_logger(), "  停止: x");
        RCLCPP_INFO(this->get_logger(), "退出: Ctrl+C");
        
        //保存终端原始配置（避免修改后终端异常）
        // 语法：tcgetattr(文件描述符, 保存配置的termios指针) → 读取终端配置
        // STDIN_FILENO：标准输入（键盘）的文件描述符（Linux中一切皆文件）
        tcgetattr(STDIN_FILENO, &original_termios_);
        
        // 启动键盘监听线程（独立线程：避免阻塞rclcpp::spin的事件循环）
        // 语法：std::thread(成员函数地址, this指针, 函数参数...)
        // 注意：成员函数需绑定this（因为成员函数依赖对象实例）
        keyboard_thread_ = std::thread(&MecanumTeleopKeyboard::keyboardLoop, this);
    }
    
    ~MecanumTeleopKeyboard()
    {
        // 恢复终端原始配置（关键！否则终端会保持「无回显+非规范模式」）
        // 语法：tcsetattr(文件描述符, 生效时机, 配置结构体指针)
        // TCSANOW：立即生效（不等待当前输入完成）
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        
        // 发送停止命令
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.angular.z = 0.0;
        cmd_vel_publisher_->publish(twist);

        // 回收键盘线程（避免线程成为「僵尸线程」）
        // 语法：thread.joinable() → 判断线程是否可回收；thread.join() → 等待线程结束
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;  //智能指针
    double linear_speed_;
    double angular_speed_;
    double strafe_speed_;
    termios original_termios_;     // 终端配置（termios结构体：存储Linux终端的原始配置）
    std::thread keyboard_thread_;   //键盘监听线程（std::thread：独立线程，避免阻塞ROS2事件循环）

    rclcpp::Time last_cmd_time_;  // 记录最后有效指令时间
    std::mutex cmd_mutex_;        // 保护指令数据的互斥锁
    
    char getKey()  //获取键盘按键（非阻塞）
    {
        char key = 0;
        termios new_termios = original_termios_;
        
        // 修改终端模式：关闭「规范模式」和「回显模式」
        // c_lflag：termios的「本地模式标志」（控制终端的本地行为）
        // ICANON：规范模式标志（关闭后，输入无需按回车即可提交）
        // ECHO：回显标志（关闭后，输入的字符不显示在屏幕上）
        // 语法：new_termios.c_lflag &= ~(标志) → 清除指定标志
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);// 应用新配置
        

        // 用select实现非阻塞等待输入（避免read函数阻塞线程）
        fd_set readfds;  // 文件描述符集合（用于监听可读事件）
        struct timeval timeout;  // 超时时间（避免无限等待
        
        FD_ZERO(&readfds);   // 清空文件描述符集合
        FD_SET(STDIN_FILENO, &readfds);  // 将「标准输入」加入监听集合
        
        timeout.tv_sec = 0;    // 秒级超时：0秒
        timeout.tv_usec = 100000; // 微秒级超时：100ms（0.1秒）
        
        // 语法：select(最大文件描述符+1, 读集合, 写集合, 异常集合, 超时)
        // 返回值：>0 → 有文件描述符就绪；=0 → 超时；<0 → 错误
        int ready = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);

        // 若有输入，读取1个字符（键盘按键通常是1个ASCII字符）
        if (ready > 0) {
            // 语法：read(文件描述符, 缓冲区, 读取字节数) → 返回读取的字节数
            read(STDIN_FILENO, &key, 1); // 从标准输入读取1个字节到key
        }
        
        // 恢复终端原始配置（每次调用后恢复，避免影响其他代码）
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        
        return key;
    }
    
    void keyboardLoop()
    {
        // 初始化最后指令时间为当前时间，避免一开始就过期
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            last_cmd_time_ = this->now();
        }
        
        while (rclcpp::ok()) {
            char key = getKey();
            auto current_time = this->now();
            
            auto twist = geometry_msgs::msg::Twist();
            bool valid_key = false;  // 标记是否为有效按键
            
            // 先处理按键（如果有的话）
            if (key != 0) {
                valid_key = true;
                
                switch (key) {
                    case 'w': // 前进
                        twist.linear.x = linear_speed_;
                        twist.linear.y = 0.0;
                        twist.angular.z = 0.0;
                        RCLCPP_INFO(this->get_logger(), "前进");
                        break;
                        
                    case 's': // 后退
                        twist.linear.x = -linear_speed_;
                        twist.linear.y = 0.0;
                        twist.angular.z = 0.0;
                        RCLCPP_INFO(this->get_logger(), "后退");
                        break;
                        
                    case 'a': // 左移
                        twist.linear.x = 0.0;
                        twist.linear.y = strafe_speed_;
                        twist.angular.z = 0.0;
                        RCLCPP_INFO(this->get_logger(), "左移");
                        break;
                        
                    case 'd': // 右移
                        twist.linear.x = 0.0;
                        twist.linear.y = -strafe_speed_;
                        twist.angular.z = 0.0;
                        RCLCPP_INFO(this->get_logger(), "右移");
                        break;
                        
                    case 'q': // 左转
                        twist.linear.x = 0.0;
                        twist.linear.y = 0.0;
                        twist.angular.z = angular_speed_;
                        RCLCPP_INFO(this->get_logger(), "左转");
                        break;
                        
                    case 'e': // 右转
                        twist.linear.x = 0.0;
                        twist.linear.y = 0.0;
                        twist.angular.z = -angular_speed_;
                        RCLCPP_INFO(this->get_logger(), "右转");
                        break;
                        
                    case 'x': // 停止
                        twist.linear.x = 0.0;
                        twist.linear.y = 0.0;
                        twist.angular.z = 0.0;
                        RCLCPP_INFO(this->get_logger(), "停止");
                        // 强制更新时间为未来，确保停止指令持续生效
                        {
                            std::lock_guard<std::mutex> lock(cmd_mutex_);
                            last_cmd_time_ = this->now() + rclcpp::Duration(1, 0); // 1秒后
                        }
                        break;
                        
                    case 3: // Ctrl+C
                        RCLCPP_INFO(this->get_logger(), "退出");
                        rclcpp::shutdown();
                        return;
                        
                    default:
                        valid_key = false;
                        break;
                }
                
                // 如果是有效按键，更新时间戳
                if (valid_key && key != 'x') {  // 'x'键已经在case中处理了时间戳
                    std::lock_guard<std::mutex> lock(cmd_mutex_);
                    last_cmd_time_ = this->now();
                }
            }
            
            // 检查指令是否过期（200ms内没有新指令就停止）
            bool cmd_expired = false;
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                auto time_diff = current_time - last_cmd_time_;
                if (time_diff > rclcpp::Duration(0, 200000000)) { // 200ms
                    cmd_expired = true;
                }
            }
            
            // 如果指令过期，强制停止（覆盖之前的指令）
            if (cmd_expired) {
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.angular.z = 0.0;
                RCLCPP_DEBUG(this->get_logger(), "指令过期，自动停止");
            }
            
            // 发布指令（总是发布，确保过期停止生效）
            cmd_vel_publisher_->publish(twist);
            
            // 控制发布频率
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
};
int main(int argc, char** argv)
{
    // 初始化ROS2环境（必须在所有ROS2操作前调用）
    // 语法：rclcpp::init(命令行参数 argc, 命令行参数 argv)
    // 作用：解析命令行参数（如--ros-args指定的参数）、初始化ROS2核心组件
    rclcpp::init(argc, argv);  //初始化
    auto node = std::make_shared<MecanumTeleopKeyboard>();    // 创建节点实例（用智能指针管理，避免内存泄漏）语法：std::make_shared<节点类>() → 创建节点的shared_ptr
    // 启动ROS2事件循环（阻塞当前线程，直到rclcpp::shutdown()被调用）
    // 作用：处理节点的回调函数（如订阅回调）、参数更新等事件（本代码无回调，但需spin保持节点运行）
    rclcpp::spin(node);      //事件循环
    rclcpp::shutdown();    //关闭ROS2环境（释放资源，如网络连接、线程等）
    return 0;
}