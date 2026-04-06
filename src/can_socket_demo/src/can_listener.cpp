// 包含ROS 2核心头文件（节点、日志、参数等功能）
#include <rclcpp/rclcpp.hpp>
// 包含C++字符串流（用于格式化CAN帧打印信息）
#include <sstream>

// 声明外部C链接：SocketCAN相关库是C语言实现的，避免C++名字修饰导致链接错误
extern "C"
{
#include <linux/can.h>        // CAN核心定义（帧结构、ID标志、掩码等）
#include <linux/can/raw.h>    // RAW模式相关定义（SocketCAN原始套接字配置）
#include <sys/types.h>        // 系统调用基础类型（如socket文件描述符类型）
#include <sys/socket.h>       // 套接字系统调用（socket、bind等）
#include <net/if.h>           // 网络接口相关结构（ifreq用于获取接口索引）
#include <sys/ioctl.h>        // IO控制调用（ioctl获取网络接口信息）
#include <unistd.h>           // unistd系统调用（close关闭文件描述符）
#include <fcntl.h>            // 文件控制调用（fcntl设置非阻塞模式）
#include <cstring>            // C字符串操作（memset、strncpy等）
#include <errno.h>            // 错误码定义（处理read等系统调用的错误）
}

// 定义CAN监听节点类，继承自ROS 2的Node基类
class CanListenerNode : public rclcpp::Node
{
public:
    // 构造函数：初始化节点、SocketCAN、定时器
    CanListenerNode() : Node("can_listener_node")  // 节点名称：can_listener_node
    {
        // 1. 声明并获取ROS 2参数（CAN接口名，默认值为"can0"）
        // 作用：允许用户通过启动参数指定CAN接口（如ros2 run xxx xxx --ros-args -p interface:=can1）
        this->declare_parameter<std::string>("interface", "can0");
        std::string if_name = this->get_parameter("interface").as_string();

        // 2. 创建SocketCAN原始套接字
        // PF_CAN：CAN协议族（SocketCAN的专用协议族）
        // SOCK_RAW：原始套接字模式（直接接收/发送完整CAN帧，不经过协议栈处理）
        // CAN_RAW：RAW模式协议类型（SocketCAN的标准RAW模式）
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0)  // 套接字创建失败（返回-1）
        {
            RCLCPP_FATAL(this->get_logger(), "socket() failed: %s", strerror(errno));
            throw std::runtime_error("Socket creation failed");
        }

        // 3. 获取CAN接口的索引（内核识别网络接口的唯一标识）
        struct ifreq ifr;  // 存储网络接口信息的结构体
        std::memset(&ifr, 0, sizeof(ifr));  // 结构体清零（避免垃圾数据）
        // 将CAN接口名（如"can0"）复制到ifr.ifr_name（最多IFNAMSIZ-1个字符，避免溢出）
        std::strncpy(ifr.ifr_name, if_name.c_str(), IFNAMSIZ - 1);

        // IO控制调用：获取接口索引（SIOCGIFINDEX为获取索引的命令）
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_FATAL(get_logger(), "ioctl(SIOCGIFINDEX) failed for %s: %s", if_name.c_str(), strerror(errno));
            close(sock_);  // 失败时关闭已创建的套接字（避免资源泄漏）
            throw std::runtime_error("IOCTL get interface index failed");
        }

        // 4. 将套接字绑定到指定CAN接口（只有绑定后才能接收该接口的CAN数据）
        std::memset(&addr_, 0, sizeof(addr_));  // CAN地址结构体清零
        addr_.can_family = AF_CAN;  // 地址族：必须为AF_CAN（与PF_CAN对应）
        addr_.can_ifindex = ifr.ifr_ifindex;  // 绑定到步骤3获取的接口索引

        // 绑定套接字（第二个参数需强制转换为通用sockaddr类型）
        if (bind(sock_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0)
        {
            RCLCPP_FATAL(get_logger(), "bind() failed: %s", strerror(errno));
            close(sock_);  // 失败时关闭套接字
            throw std::runtime_error("Socket bind failed");
        }

        // 5. 设置套接字为非阻塞模式（关键：避免read()调用阻塞节点）
        int flags = fcntl(sock_, F_GETFL, 0);  // 获取当前套接字状态标志
        fcntl(sock_, F_SETFL, flags | O_NONBLOCK);  // 添加非阻塞标志（O_NONBLOCK）

        // 打印初始化成功日志
        RCLCPP_INFO(get_logger(), "CAN socket initialized successfully! Bound to interface: %s", if_name.c_str());

        // 6. 创建ROS 2定时器：定时读取CAN数据（5毫秒触发一次）
        // 绑定回调函数poll_can（每隔5ms调用一次，查询CAN数据）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),  // 定时器周期：5ms
            std::bind(&CanListenerNode::poll_can, this)  // 回调函数绑定（this指针传入成员函数）
        );
    }

    // 析构函数：释放资源（关闭CAN套接字）
    ~CanListenerNode()
    {
        if (sock_ >= 0)  // 确保套接字描述符有效（避免重复关闭）
        {
            close(sock_);  // 关闭套接字，释放内核资源
            RCLCPP_INFO(get_logger(), "CAN socket closed");
        }
    }

private:
    int sock_;  // CAN套接字文件描述符（用于操作套接字的句柄）
    struct sockaddr_can addr_;  // CAN地址结构体（存储绑定的接口信息）
    rclcpp::TimerBase::SharedPtr timer_;  // ROS 2定时器指针（控制定时读取）

    /**
     * @brief 定时器回调函数：读取CAN数据并解析打印
     * 功能：非阻塞读取CAN帧 -> 错误处理 -> 解析帧信息 -> 格式化打印
     */
    void poll_can()
    {
        struct can_frame frame;  // 存储接收的CAN帧（定义在linux/can.h）
        // 读取CAN数据：从sock_套接字读取一个can_frame大小的数据
        int nbytes = read(sock_, &frame, sizeof(frame));

        // 情况1：读取失败（nbytes < 0）
        if (nbytes < 0)
        {
            // 非阻塞模式下"无数据可读"是正常情况（errno=EAGAIN/EWOULDBLOCK），直接返回
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                return;
            }
            // 其他错误（如接口断开）：打印警告日志
            else
            {
                RCLCPP_WARN(get_logger(), "read() error: %s", strerror(errno));
                return;
            }
        }

        // 情况2：读取数据不完整（接收到的字节数小于CAN帧大小）
        if (nbytes < (int)sizeof(struct can_frame))
        {
            RCLCPP_WARN(get_logger(), "Incomplete CAN frame received! Bytes read: %d (expected: %zu)", nbytes, sizeof(struct can_frame));
            return;
        }

        // --------------- 解析CAN帧关键信息 ---------------
        bool is_ext = frame.can_id & CAN_EFF_FLAG;  // 是否为扩展帧（29位ID）：CAN_EFF_FLAG是扩展帧标志位
        bool is_rtr = frame.can_id & CAN_RTR_FLAG;  // 是否为远程请求帧（RTR）：无数据，仅请求数据
        bool is_err = frame.can_id & CAN_ERR_FLAG;  // 是否为错误帧：记录CAN总线错误

        // 提取真实CAN ID（去除标志位掩码）
        // CAN_EFF_MASK：扩展帧ID掩码（29位），CAN_SFF_MASK：标准帧ID掩码（11位）
        uint32_t id = frame.can_id & (is_ext ? CAN_EFF_MASK : CAN_SFF_MASK);

        // --------------- 格式化打印信息 ---------------
        std::stringstream ss;  // 字符串流：拼接打印内容（高效且易维护）
        ss << "Received CAN Frame: "
           << "ID=0x" << std::hex << std::uppercase << id  // ID（十六进制大写）
           << (is_ext ? " (EXT)" : " (STD)")  // 帧类型（扩展/标准）
           << (is_rtr ? " [RTR]" : "")  // 是否为远程帧
           << (is_err ? " [ERROR]" : "")  // 是否为错误帧
           << ", DLC=" << std::dec << (int)frame.can_dlc  // DLC（数据长度码：0-8）
           << ", Data=[";  // 数据段开始

        // 拼接数据段（逐个字节打印，十六进制）
        for (int i = 0; i < frame.can_dlc; ++i)
        {
            ss << "0x" << std::hex << std::uppercase << (int)frame.data[i];  // 每个数据字节转十六进制
            if (i != frame.can_dlc - 1)  // 非最后一个字节，添加空格分隔
                ss << " ";
        }
        ss << "]";  // 数据段结束

        // 打印格式化后的CAN帧信息（ROS 2日志级别：INFO）
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }
};

// 主函数：ROS 2节点入口
int main(int argc, char *argv[])
{
    // 1. 初始化ROS 2上下文（解析命令行参数、初始化内核资源）
    rclcpp::init(argc, argv);
    // 2. 创建CAN监听节点实例（智能指针：自动管理内存，避免内存泄漏）
    auto node = std::make_shared<CanListenerNode>();
    // 3. 启动ROS 2节点循环（阻塞，直到节点被关闭）
    rclcpp::spin(node);
    // 4. 关闭ROS 2上下文（释放资源）
    rclcpp::shutdown();
    return 0;
}