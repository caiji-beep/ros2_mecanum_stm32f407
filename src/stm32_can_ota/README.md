# stm32_can_ota

`stm32_can_ota` 是 STM32 Bootloader CAN OTA 的 ROS2 上位机功能包骨架。

当前版本只建立代码架构，不实现完整 OTA 流程。后续真实刷写逻辑应放在 `OtaClient` 状态机中，并通过 `CanTransport` 与 SocketCAN 通信。

## 1. 设计目标

上位机只负责：

- 读取固件包
- 查询 Bootloader capability
- 按协议发送 OTA 请求和数据
- 发布 OTA 状态
- 暴露 ROS2 service 触发升级

上位机不应该关心 STM32 内部升级策略，例如：

- 单区直接写 App
- 双区暂存
- A/B 分区
- 回滚策略
- Flash 擦除页布局

这些信息应由 Bootloader 通过 `GET_CAPABILITY` 返回，例如：

```text
target_slot
required_image
max_fw_size
block_size
protocol_version
```

## 2. 目录结构

```text
stm32_can_ota/
├── CMakeLists.txt
├── package.xml
├── include/stm32_can_ota/
│   ├── stm32_ota_node.hpp
│   ├── ota_client.hpp
│   ├── ota_state.hpp
│   ├── ota_error.hpp
│   ├── can_transport.hpp
│   ├── can_frame.hpp
│   ├── ota_protocol.hpp
│   ├── firmware_image.hpp
│   ├── firmware_package.hpp
│   ├── crc16.hpp
│   └── crc32.hpp
├── src/
│   ├── main.cpp
│   ├── stm32_ota_node.cpp
│   ├── ota_client.cpp
│   ├── can_transport.cpp
│   ├── ota_protocol.cpp
│   ├── firmware_image.cpp
│   ├── firmware_package.cpp
│   ├── crc16.cpp
│   └── crc32.cpp
├── srv/
│   └── StartOta.srv
├── msg/
│   └── OtaStatus.msg
├── config/
│   └── ota.yaml
└── launch/
    └── stm32_can_ota.launch.py
```

## 3. 分层职责

### `stm32_ota_node`

ROS2 节点层。

- 提供 `/stm32_ota/start` service
- 发布 `/stm32_ota/status` topic
- 读取 ROS2 参数
- 将 ROS 请求转给 `OtaClient`

### `ota_client`

OTA 状态机层。

- 管理 OTA 状态
- 加载固件包
- 后续执行 `SYNC -> GET_CAPABILITY -> START -> BLOCK_BEGIN -> DATA -> END`
- 不直接操作 SocketCAN
- 只通过 `CanTransport` 收发 CAN 帧

当前版本中，`dry_run=true` 会加载固件并计算 CRC32；真实 OTA 传输暂时返回 `NOT_IMPLEMENTED`。

### `can_transport`

SocketCAN 封装层。

- `open`
- `close`
- `send`
- `receive`
- `receiveById`

### `ota_protocol`

CAN OTA 协议封装层。

预留命令：

```text
SYNC
GET_CAPABILITY
START
BLOCK_BEGIN
DATA
END
ACK
NACK
```

### `firmware_image`

固件镜像读取层。

- 读取 `.bin`
- 计算 CRC32
- 按 offset/block 读取数据

### `firmware_package`

固件包层。

- v0.1 只支持单个 `.bin`
- 后续可扩展 `manifest.json`
- 当前不依赖第三方 JSON 库

## 4. 编译

```bash
cd /home/lsz/robotmini_ws
colcon build --packages-select stm32_can_ota
source install/setup.bash
```

注意：每打开一个新终端，都要重新执行：

```bash
cd /home/lsz/robotmini_ws
source install/setup.bash
```

否则 ROS2 找不到本包自定义的 service/msg，调用 service 时会报：

```text
The passed service type is invalid
```

可以用下面命令确认当前终端是否已经识别 `StartOta`：

```bash
ros2 interface show stm32_can_ota/srv/StartOta
```

正常输出应为：

```text
string firmware_path
uint32 firmware_version
bool dry_run
---
bool accepted
uint8 state
uint8 error_code
string message
```

## 5. 从网络下载固件到本地

开发阶段可以先把 `.bin` 放在 GitHub Releases，然后用脚本下载到上位机本地缓存。

默认缓存目录：

```text
~/.stm32_can_ota/firmware/
```

下载示例：

```bash
ros2 run stm32_can_ota download_firmware.py \
  https://raw.githubusercontent.com/caiji-beep/ros2_mecanum_stm32f407/stm32f407/release/app/slave_app.bin \
  --filename slave_app.bin
```

脚本会输出实际保存路径，并更新：

```text
~/.stm32_can_ota/firmware/latest.bin
```

如果你已经知道 SHA256，可以加校验：

```bash
ros2 run stm32_can_ota download_firmware.py \
  https://github.com/<owner>/<repo>/releases/download/<tag>/app_A.bin \
  --sha256 <expected_sha256>
```

之后 OTA service 可以使用本地缓存文件：

```bash
ros2 service call /stm32_ota/start stm32_can_ota/srv/StartOta \
  "{firmware_path: '/home/lsz/.stm32_can_ota/firmware/latest.bin', firmware_version: 1, dry_run: true}"
```

当前建议先固定流程：

```text
GitHub Releases -> 本地 latest.bin -> dry_run -> 后续真实 CAN OTA
```

## 6. 启动

```bash
ros2 launch stm32_can_ota stm32_can_ota.launch.py
```

指定 CAN 参数：

```bash
ros2 launch stm32_can_ota stm32_can_ota.launch.py \
  can_interface:=can0 \
  node_id:=1 \
  request_base_id:=1536 \
  response_base_id:=1408 \
  extended_id:=false
```

默认 CAN ID：

```text
上位机 -> STM32: 0x600 + node_id
STM32 -> 上位机: 0x580 + node_id
```

## 7. 配置参数

默认配置位于：

```text
config/ota.yaml
```

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `can_interface` | `can0` | SocketCAN 接口 |
| `node_id` | `1` | 目标 STM32 节点 ID |
| `request_base_id` | `1536` | 请求基础 CAN ID，十六进制为 `0x600` |
| `response_base_id` | `1408` | 应答基础 CAN ID，十六进制为 `0x580` |
| `extended_id` | `false` | 是否使用扩展帧 |
| `default_firmware_path` | `""` | service 未传路径时使用的默认固件 |
| `block_size` | `256` | OTA block 大小，后续应以 Bootloader capability 为准 |
| `ack_timeout_ms` | `1000` | ACK 超时 |
| `capability_timeout_ms` | `1000` | capability 查询超时 |
| `receive_timeout_ms` | `100` | CAN 接收轮询超时 |
| `status_publish_period_ms` | `500` | 状态发布周期 |

## 8. Service

服务名：

```text
/stm32_ota/start
```

类型：

```text
stm32_can_ota/srv/StartOta
```

请求：

```text
string firmware_path
uint32 firmware_version
bool dry_run
```

响应：

```text
bool accepted
uint8 state
uint8 error_code
string message
```

dry run 示例：

先启动节点：

```bash
cd /home/lsz/robotmini_ws
source install/setup.bash
ros2 launch stm32_can_ota stm32_can_ota.launch.py
```

再打开第二个终端调用 service：

```bash
cd /home/lsz/robotmini_ws
source install/setup.bash
ros2 service call /stm32_ota/start stm32_can_ota/srv/StartOta \
  "{firmware_path: '/home/lsz/.stm32_can_ota/firmware/latest.bin', firmware_version: 1, dry_run: true}"
```

正常响应示例：

```text
response:
stm32_can_ota.srv.StartOta_Response(
  accepted=True,
  state=3,
  error_code=0,
  message='dry run complete; firmware image loaded, no CAN frames sent')
```

## 9. Status Topic

Topic：

```text
/stm32_ota/status
```

类型：

```text
stm32_can_ota/msg/OtaStatus
```

查看：

```bash
ros2 topic echo /stm32_ota/status
```

状态字段中包含：

```text
state
error_code
progress
firmware_path
image_size
image_crc32
target_slot
required_image
max_fw_size
block_size
message
```

## 10. 当前限制

当前只是骨架包：

- 未实现真实 OTA 数据传输
- 未实现 Bootloader capability 查询
- 未实现 ACK/NACK 超时重传
- 未实现 manifest.json
- 未实现并发升级保护
- 未实现固件签名校验

下一步建议先确定 STM32 Bootloader 的 `GET_CAPABILITY` ACK 数据格式，再补 `OtaProtocol::parseCapability()` 和 `OtaClient` 状态机。

## 11. 常见问题

### `The passed service type is invalid`

原因通常是当前终端没有 source 工作区环境，ROS2 找不到 `stm32_can_ota/srv/StartOta`。

处理：

```bash
cd /home/lsz/robotmini_ws
source install/setup.bash
ros2 interface show stm32_can_ota/srv/StartOta
```

如果 `ros2 interface show` 仍然找不到接口，重新编译：

```bash
cd /home/lsz/robotmini_ws
colcon build --packages-select stm32_can_ota
source install/setup.bash
```

### 一直显示 `waiting for service to become available...`

原因是 `/stm32_ota/start` 节点没有启动。

先在另一个终端启动：

```bash
cd /home/lsz/robotmini_ws
source install/setup.bash
ros2 launch stm32_can_ota stm32_can_ota.launch.py
```

确认 service 是否存在：

```bash
ros2 service list | grep stm32_ota
```

应能看到：

```text
/stm32_ota/start
```
