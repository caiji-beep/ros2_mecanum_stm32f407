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

该包实现了两个 ROS2 节点：

- `can_listener`：绑定指定 CAN 接口，非阻塞读取原始帧，打印 CAN ID、帧类型、DLC 和数据区。
- `can_imu_node`：解析下位机 IMU CAN 帧，发布 `sensor_msgs/Imu`。

它的目标是先确认：

> 真机上到底有没有 CAN 数据进来。

---

## 2. 目录结构

```text
can_socket_demo/
├── launch/
│   └── can_imu.launch.py
├── src/
│   ├── can_imu_node.cpp
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

启动 CAN IMU：

```bash
ros2 launch can_socket_demo can_imu.launch.py \
  namespace:=robmini \
  tf_prefix:=robmini
```

`can_imu_node` 默认订阅 CAN ID：

| CAN ID | 内容 |
| --- | --- |
| `0x180` | 三轴线加速度 |
| `0x181` | 三轴角速度 |
| `0x182` | 三轴姿态角，同时触发 IMU 消息发布 |
| `0x184` | IMU 状态 |

`0x180`、`0x181`、`0x182` 均按下位机协议使用 8 字节帧：前 6 字节是三轴 `int16` 小端数据，第 6 字节是 `seq`，第 7 字节是状态 flags。节点会检查 `data_valid` 和 `fault` 标志，并要求同一组加速度、角速度、姿态角的 `seq` 一致后才发布 IMU 消息。

下位机单位约定：

- 加速度：`ax/ay/az = g * 1000`
- 角速度：`gx/gy/gz = rad/s * 1000`
- 姿态角：`roll/pitch/yaw = rad * 1000`

默认不把下位机欧拉角发布为 IMU orientation，避免无磁力计约束的 yaw 漂移被上位机当成绝对航向融合。此时 `orientation_covariance[0] = -1`，后续 EKF 应只使用角速度和需要的线加速度。

调试下位机欧拉角时可以临时打开：

```bash
ros2 launch can_socket_demo can_imu.launch.py \
  namespace:=robmini \
  tf_prefix:=robmini \
  publish_orientation:=true
```

如果下位机角速度或欧拉角单位是度，需要显式指定单位：

```bash
ros2 launch can_socket_demo can_imu.launch.py \
  namespace:=robmini \
  tf_prefix:=robmini \
  gyro_unit:=deg_per_s \
  orientation_unit:=deg
```

如果希望只有下位机标记 `calibrated` 后才发布 IMU 消息：

```bash
ros2 launch can_socket_demo can_imu.launch.py \
  namespace:=robmini \
  tf_prefix:=robmini \
  require_calibrated:=true
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
