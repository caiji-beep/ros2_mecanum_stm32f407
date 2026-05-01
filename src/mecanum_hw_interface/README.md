# mecanum_hw_interface 功能包说明

`mecanum_hw_interface` 是真实麦克纳姆轮底盘的 `ros2_control` 硬件接口插件。

它的作用不是直接接收 `/cmd_vel`，也不是计算麦克纳姆运动学，而是站在 controller 和 STM32 下位机之间：

```text
/cmd_vel
  -> mecanum_drive_controller
  -> ros2_control_node
  -> mecanum_hw_interface
  -> 串口 /dev/ttySTM32
  -> STM32F407
  -> 电机驱动
```

一句话：上层 controller 负责“算四个轮子该转多快”，这个包负责“把四个轮速发给 STM32，并把 STM32 回传的四个轮速交回 ros2_control”。

## 这个包解决什么问题

真实机器人运行 Nav2 时，数据链路大致是：

```text
Nav2 输出 /cmd_vel
controller 把底盘速度转换成四个轮子的角速度
hardware interface 把四个轮子的角速度通过串口发给 STM32
STM32 控制电机，并回传四个轮子的测量角速度
hardware interface 把测量值作为 joint state 反馈给 ros2_control
controller 根据反馈发布里程计 /odom
```

所以这个包是“真实硬件接入层”。如果它没加载成功，机器人不会真正动；如果它读回的轮速不对，里程计和导航也会跟着不准。

## 它不是哪些东西

这个包容易和另外几个概念混在一起：

```text
mecanum_hw_interface
  真实硬件接口，负责串口收发和 ros2_control 的 read/write。

mecanum_drive_controller
  控制器，负责把 cmd_vel 转成四个轮子的速度，并根据轮速反馈生成 odom。

robmini_description
  机器人模型、ros2_control 的 URDF 配置、controller 参数。

robmini_navigation
  Nav2、地图、雷达、RViz、SLAM 等导航相关启动文件。
```

通常你不会直接 `ros2 run mecanum_hw_interface ...`，因为它不是普通节点，而是被 `ros2_control_node` 通过 pluginlib 动态加载。

## 文件结构

```text
mecanum_hw_interface/
|-- include/mecanum_hw_interface/
|   |-- mecanum_hw_interface.hpp     # ros2_control SystemInterface 声明
|   `-- mecanum_serial_port.hpp      # 串口协议封装声明
|-- src/
|   |-- mecanum_hw_interface.cpp     # 硬件接口主体：on_init/read/write
|   `-- mecanum_serial_port.cpp      # 打开串口、打包、解包、CRC
|-- test/
|-- mecanum_hw_interface.xml         # pluginlib 插件声明
|-- CMakeLists.txt
|-- package.xml
`-- README.md
```

## ros2_control 怎么加载它

插件声明在：

```text
mecanum_hw_interface.xml
```

里面导出的插件名是：

```text
mecanum_hw_interface/MecanumHWSystem
```

URDF/xacro 中通过下面这段配置加载它：

```xml
<ros2_control name="mecanum_hw_system" type="system">
  <hardware>
    <plugin>mecanum_hw_interface/MecanumHWSystem</plugin>
    <param name="device">/dev/ttySTM32</param>
    <param name="baud">115200</param>
  </hardware>
  ...
</ros2_control>
```

当前项目中，这段配置位于：

```text
robmini_description/urdf/robmini.ros2_control.xacro
```

真实机器人启动时，`real_bringup.launch.py` 会启动 `ros2_control_node`，然后 `ros2_control_node` 读取 robot_description，最终加载这个硬件插件。

## 对外提供的 joint 接口

这个包要求 URDF 里必须有 4 个轮子关节，名称必须完全一致：

```text
front_left_wheel_joint
front_right_wheel_joint
rear_right_wheel_joint
rear_left_wheel_joint
```

每个关节都必须提供：

```text
command interface:
  velocity

state interface:
  velocity
  position
```

含义如下：

```text
velocity command
  controller 写入的目标轮子角速度，单位 rad/s。

velocity state
  STM32 回传的实际轮子角速度，单位 rad/s。

position state
  当前代码用 velocity 积分得到轮子角位置，单位 rad。
```

如果后续 STM32 能直接回传编码器位置，可以把 `position state` 改成真实编码器角位置。

## 四个轮子的顺序

这是最容易出错的地方，必须和 STM32 固件保持一致。

本包内部 joint 名称是：

```text
front_left_wheel_joint   -> FL，左前轮
front_right_wheel_joint  -> FR，右前轮
rear_right_wheel_joint   -> RR，右后轮
rear_left_wheel_joint    -> RL，左后轮
```

但是与 STM32 通讯时，当前协议里的 4 个 float 顺序是：

```text
[FR, FL, RL, RR]
```

也就是说，发给 STM32 的命令包载荷是：

```text
float0 = front_right_wheel_joint
float1 = front_left_wheel_joint
float2 = rear_left_wheel_joint
float3 = rear_right_wheel_joint
```

从 STM32 读回测量值时也按这个顺序解析：

```text
float0 -> FR
float1 -> FL
float2 -> RL
float3 -> RR
```

如果出现前进变后退、横移方向反了、原地旋转方向不对，优先检查这里和 STM32 固件里的轮序、符号是否一致。

## 串口参数

当前真实机器人默认串口配置是：

```text
device = /dev/ttySTM32
baud   = 115200
```

代码支持的波特率：

```text
115200
230400
460800
921600
```

串口模式：

```text
8N1
raw mode
non-blocking
关闭软件流控
关闭硬件流控
```

如果设备名变了，不建议直接改 C++ 代码，应该改 URDF/xacro 里的参数：

```xml
<param name="device">/dev/ttySTM32</param>
<param name="baud">115200</param>
```

## 串口协议

### 命令包：ROS2 发给 STM32

```text
AA 55 01 10 + 4 个 float32 小端 + CRC16
```

字段说明：

```text
AA 55       包头
01          包 ID，表示轮速命令
10          数据长度，十六进制 0x10，也就是 16 字节
float0      FR 轮角速度，rad/s
float1      FL 轮角速度，rad/s
float2      RL 轮角速度，rad/s
float3      RR 轮角速度，rad/s
CRC16       对前 20 字节做 CRC 校验，低字节在前
```

总长度：

```text
4 字节包头 + 16 字节数据 + 2 字节 CRC = 22 字节
```

### 测量包：STM32 发给 ROS2

```text
AA 55 02 10 + 4 个 float32 小端 + CRC16
```

字段说明：

```text
AA 55       包头
02          包 ID，表示轮速测量
10          数据长度，16 字节
float0      FR 轮实际角速度，rad/s
float1      FL 轮实际角速度，rad/s
float2      RL 轮实际角速度，rad/s
float3      RR 轮实际角速度，rad/s
CRC16       对前 20 字节做 CRC 校验，低字节在前
```

CRC 使用 Modbus 风格 CRC16：

```text
初始值：0xFFFF
多项式：0xA001
```

## 启动方式

通常不单独启动这个包，而是启动真实机器人 bringup：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py
```

或者只启动底盘硬件和 controller：

```bash
ros2 launch robmini_description real_bringup.launch.py
```

启动后检查 controller：

```bash
ros2 control list_controllers -c /robmini/controller_manager
```

正常应该看到类似：

```text
joint_state_broadcaster active
mecanum_drive_controller active
```

检查硬件接口：

```bash
ros2 control list_hardware_interfaces -c /robmini/controller_manager
```

应该能看到四个轮子的 `velocity` command interface，以及 `velocity`、`position` state interface。

## 最小测试流程

### 1. 确认串口设备存在

```bash
ls -l /dev/ttySTM32
```

如果不存在，先检查 USB、udev 规则、设备映射。

### 2. 确认权限

```bash
groups
```

通常用户需要在 `dialout` 组里：

```bash
sudo usermod -aG dialout $USER
```

执行后需要重新登录终端或重启。

### 3. 启动底盘 bringup

```bash
ros2 launch robmini_description real_bringup.launch.py
```

注意终端里是否出现：

```text
Serial opened.
```

如果出现：

```text
Serial open failed
```

说明硬件接口插件加载了，但是串口没打开，需要检查设备名、权限、线缆、波特率。

### 4. 发一个低速命令

```bash
ros2 topic pub /robmini/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}" --once
```

如果 controller active、串口正常、STM32 协议正确，底盘应该低速前进一下。

### 5. 看 joint state

```bash
ros2 topic echo /robmini/joint_states
```

如果 STM32 正常回传测量包，轮子的 velocity 应该会变化。

## 常见问题

### 1. `Serial open failed`

常见原因：

- `/dev/ttySTM32` 不存在。
- 当前用户没有串口权限。
- udev 规则没有生效。
- 实际设备名是 `/dev/ttyUSB0` 或 `/dev/ttyACM0`。
- STM32 没上电或 USB 线异常。

### 2. controller 是 active，但底盘不动

按顺序查：

```text
Nav2 或手动命令是否发到了 /robmini/cmd_vel
mecanum_drive_controller 是否 active
hardware interface 是否打开了串口
STM32 是否收到 AA 55 01 10 命令包
STM32 是否通过 CRC 校验
电机驱动是否使能
```

### 3. 底盘能动，但方向不对

优先检查：

```text
四个轮子的顺序是否一致
某个轮子的正负号是否反了
STM32 固件里的电机编号是否和 ROS 侧一致
mecanum_drive_controller 配置中的 joint name 是否一致
```

特别注意当前串口协议顺序是：

```text
[FR, FL, RL, RR]
```

### 4. 能动，但里程计飘得厉害

常见原因：

- STM32 回传的是目标速度，不是真实编码器速度。
- 轮速单位不是 `rad/s`。
- 轮子半径或底盘几何参数不准。
- 某个轮子的反馈方向反了。
- 串口丢包或回传频率太低。

### 5. 没有硬件时能不能调上层

这个包串口打开失败时不会让 `ros2_control_node` 直接退出，而是打印警告并继续运行。
这样方便先调 controller、URDF、Nav2 等上层部分。

不过没有串口测量反馈时，真实闭环效果不能代表最终底盘效果。

## 和命名空间的关系

这个包本身不创建普通 ROS topic，也不直接处理 namespace。

命名空间由外层 `ros2_control_node` 和 controller 决定，例如：

```text
/robmini/controller_manager
/robmini/mecanum_drive_controller
/robmini/cmd_vel
/robmini/odom
```

多机器人时，每台机器人都会有自己的 `controller_manager`，但它们可以加载同一个插件类：

```text
/robot1/controller_manager -> mecanum_hw_interface/MecanumHWSystem
/robot2/controller_manager -> mecanum_hw_interface/MecanumHWSystem
```

注意：如果多台真实机器人在同一台主机上运行，串口设备名必须分别配置，不能都用同一个 `/dev/ttySTM32`。

## 后续建议

可以继续优化的方向：

- 让 STM32 回传真实编码器位置，替代当前由速度积分得到 position。
- 给串口协议增加心跳、错误码或急停状态。
- 统计 CRC 错误次数和丢包次数，便于现场排障。
- 把轮序 `[FR, FL, RL, RR]` 做成明确协议文档，并同步到 STM32 工程。
- 为 `/dev/ttySTM32` 建立稳定 udev 规则，避免 USB 顺序变化导致设备名变动。
