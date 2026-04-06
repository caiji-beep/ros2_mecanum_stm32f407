# mecanum_hw_interface（humble_real 分支）

`mecanum_hw_interface` 是 `humble_real` 分支中最核心的实机接口包。它通过 `ros2_control` 将上层控制器与真实 STM32 下位机连接起来，使 ROS2 能够把速度命令真正下发到底盘，并从下位机回读轮速反馈。

---

## 1. 包定位

该包处于实机控制闭环的中间层：

```text
/cmd_vel 或导航输出
        ↓
mecanum_drive_controller
        ↓
mecanum_hw_interface
        ↓
串口协议
        ↓
STM32F407
        ↓
麦克纳姆轮底盘
```

因此，它既不是简单的串口工具，也不是纯控制器配置，而是 **实机控制落地的关键桥梁**。

---

## 2. 目录结构

```text
mecanum_hw_interface/
├── include/mecanum_hw_interface/
│   ├── mecanum_hw_interface.hpp
│   └── mecanum_serial_port.hpp
├── src/
│   ├── mecanum_hw_interface.cpp
│   └── mecanum_serial_port.cpp
├── test/
├── CMakeLists.txt
├── mecanum_hw_interface.xml
└── package.xml
```

---

## 3. 主要功能

### 3.1 插件方式接入 `ros2_control`

通过 `pluginlib` 导出硬件接口插件，由 `ros2_control_node` 在 bringup 时动态加载。

### 3.2 下发四轮速度命令

控制器输出的命令会被整理成四轮速度数组，随后通过串口协议发往 STM32。

### 3.3 读取四轮测量反馈

STM32 回传 4 个浮点数轮速，供状态接口输出与控制器使用。

### 3.4 串口异常下的容错运行

如果设备打开失败，程序会打印告警并继续运行，有利于开发阶段的分层调试。

---

## 4. 串口协议

### 4.1 命令帧格式

```text
AA 55 01 10 + 4 * float32(LE) + CRC16
```

### 4.2 测量帧格式

```text
AA 55 02 10 + 4 * float32(LE) + CRC16
```

### 4.3 校验方式

采用 CRC-16/IBM（Modbus 风格），初始化值 `0xFFFF`，多项式 `0xA001`。

### 4.4 总长度

- 头部：4 字节
- 载荷：16 字节
- CRC：2 字节
- 总计：22 字节

---

## 5. 轮序与关节命名

在实机项目里，下面几件事必须统一：

- 控制器 YAML 中的轮子关节名
- 硬件接口内部命令/状态数组顺序
- STM32 协议中 4 个浮点数的轮序
- 底盘驱动层真实电机编号

推荐在 README 中长期明确标注轮序，不要只靠代码记忆，否则后续非常容易出现左右反、前后反、旋转方向异常的问题。

---

## 6. 编译

```bash
colcon build --packages-select mecanum_hw_interface
source install/setup.bash
```

---

## 7. 使用方式

这个包通常不直接运行，而是通过 bringup launch 间接加载：

```bash
ros2 launch robmini_description real_bringup.launch.py
```

在这个过程中，`ros2_control_node` 会：

- 加载机器人描述
- 读取控制器参数
- 加载该硬件接口插件
- 启动关节状态广播器和麦克纳姆控制器

---

## 8. 与下位机联调建议

### 阶段 1：虚拟串口联调

先确认协议和 CRC 都正确。

### 阶段 2：实机空载联调

接真实 STM32，但先不接导航，只发简单速度指令。

### 阶段 3：轮速反馈校验

确认前进、横移、旋转三种运动时的四轮反馈是否合理。

### 阶段 4：挂载导航系统

在底盘控制链路完全正常之后，再接 Nav2。

---

## 9. 常见问题

### 串口能打开，但底盘不动

通常要从以下几方面排查：

- 控制器是否真的收到了 `cmd_vel`
- 串口命令是否真的发出
- STM32 是否解析成功
- 电机使能是否正常

### 底盘能动，但方向不对

优先检查轮序和符号。

### 导航时运动异常

先不要直接怀疑 Nav2，很多时候是底层轮速反馈或里程计建模不对。

---

## 10. 项目表述示例

> 基于 ROS2 `ros2_control` 实现实机麦克纳姆轮底盘自定义硬件接口，完成四轮速度命令下发、STM32 串口协议解析、轮速状态反馈与控制器插件化加载，支撑真机导航与运动控制闭环运行。