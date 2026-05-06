# can_socket_demo

`can_socket_demo` 是底层调试辅助包，用于在真机环境下快速验证 **Linux 端 CAN 接口是否正常收帧**。

如果你的系统中存在：

- STM32F103 / STM32F4 / 其他控制板
- USB-CAN 设备
- 工控机 CAN 接口
- 机器人底盘 CAN 总线

那么这个包可以作为最先使用的排查工具之一。

---

## 1. 包作用

该包实现了一个最小 ROS2 节点：

- 绑定指定 CAN 接口
- 非阻塞读取原始帧
- 打印 CAN ID、帧类型、DLC 和数据区

它的目标是先确认：

> 真机上到底有没有 CAN 数据进来。

---

## 2. 目录结构

```text
can_socket_demo/
├── src/
│   └── can_listener.cpp
├── CMakeLists.txt
└── package.xml
```

---

## 3. 使用场景

### 场景 A：实机 CAN 总线联调

用于确认某块控制板是否真的在发数据。

### 场景 B：系统升级后排查接口异常

用于确认问题是在驱动层还是应用层。

### 场景 C：导航主链路之外的辅助调试

该包独立、简单，不会与导航主逻辑耦合。

---

## 4. 使用方法

### 启动 CAN 接口

```bash
sudo ip link set can0 up type can bitrate 125000
```

### 编译

```bash
colcon build --packages-select can_socket_demo
source install/setup.bash
```

### 运行

```bash
ros2 run can_socket_demo can_listener
```

指定接口：

```bash
ros2 run can_socket_demo can_listener --ros-args -p interface:=can1
```

---

## 5. 输出信息说明

节点会打印：

- ID
- 标准帧 / 扩展帧
- RTR / ERROR 标记
- DLC
- 数据区字节

它不负责业务协议解码，因此非常适合作为“最底层的可见性工具”。

---

## 6. 维护建议

如果后续真机系统会更多依赖 CAN，建议进一步扩展：

- 发帧节点
- 过滤器
- 指定 ID 统计
- 协议级解析
- 日志持久化
