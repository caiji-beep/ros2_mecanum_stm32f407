# stm32_can_ota

`stm32_can_ota` 是 STM32 Bootloader CAN OTA 的 ROS2 上位机功能包。

当前上位机已实现经典 CAN 分块传输、异步状态机、CRC16/CRC32、ACK/NACK 超时重试、
并发保护和取消。下位机应按照 [经典 CAN OTA 协议 v1](docs/protocol_v1.md) 实现接收端。

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
max_fw_size
block_size
protocol_version
```

`required_image` 目前是主机侧展示字段；若以后需要由下位机指定，可增加 capability 扩展页。

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
│   ├── StartOta.srv
│   └── CancelOta.srv
├── msg/
│   └── OtaStatus.msg
├── config/
│   └── ota.yaml
├── docs/
│   └── protocol_v1.md
└── launch/
    └── stm32_can_ota.launch.py
```

## 3. 分层职责

### `stm32_ota_node`

ROS2 节点层。

- 提供 `/stm32_ota/start` service
- 提供 `/stm32_ota/cancel` service
- 发布 `/stm32_ota/status` topic
- 读取 ROS2 参数
- 将 ROS 请求转给 `OtaClient`

### `ota_client`

OTA 状态机层。

- 管理 OTA 状态
- 加载固件包
- 执行 `SYNC -> GET_CAPABILITY -> START -> START_CRC -> START_VERSION -> BLOCK_BEGIN -> DATA -> BLOCK_END -> END`
- 不直接操作 SocketCAN
- 只通过 `CanTransport` 收发 CAN 帧

`dry_run=true` 只加载固件并计算 CRC32；`dry_run=false` 启动后台 CAN OTA 任务。

### `can_transport`

SocketCAN 封装层。

- `open`
- `close`
- `send`
- `receive`
- `receiveById`

### `ota_protocol`

CAN OTA 协议封装层。

命令码、请求帧布局、Capability 分页应答的逐字节图与实例，见
[经典 CAN OTA 协议 v1](docs/protocol_v1.md)（重点看 §4.1「逐字节布局图」和 §4.2「完整交互实例」，
下位机对接时务必按其中的字节偏移填充，否则会解析错位）。

实现的命令：

```text
SYNC
GET_CAPABILITY
START
START_CRC
START_VERSION
BLOCK_BEGIN
DATA
BLOCK_END
END
ABORT
ACK
NACK
```

### `firmware_image`

固件镜像读取层。

- 读取 `.bin`
- 计算 CRC32
- 按 offset/block 读取数据

### `firmware_package`

固件包层，由 `FirmwareImage`（数据）+ `FirmwarePackageManifest`（元信息）组成。

- 支持直接加载单个 `.bin`（`loadSingleImage`，默认镜像槽 `app_A`、版本 0）
- 也支持从清单加载（`loadManifest`，清单里可指定 `image_path` / `required_image` / `firmware_version`）
- 当前清单为内部结构体解析，不依赖第三方 JSON 库

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
string resolved_firmware_path
uint32 image_size
uint32 image_crc32
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
  https://raw.githubusercontent.com/caiji-beep/ros2_mecanum_stm32f407/stm32f407/release/app/slave_app.bin \
  --filename slave_app.bin \
  --sha256 <expected_sha256>
```

之后 OTA service 可以使用本地缓存文件。当前 launch 默认会把 `default_firmware_path` 设置为：

```text
$HOME/.stm32_can_ota/firmware/latest.bin
```

所以 service 请求里的 `firmware_path` 可以留空：

```bash
ros2 service call /stm32_ota/start stm32_can_ota/srv/StartOta \
  "{firmware_path: '', firmware_version: 1, dry_run: true}"
```

也可以显式传入路径，显式路径优先级更高：

```bash
ros2 service call /stm32_ota/start stm32_can_ota/srv/StartOta \
  "{firmware_path: '/home/lsz/.stm32_can_ota/firmware/latest.bin', firmware_version: 1, dry_run: true}"
```

当前建议先固定流程：

```text
GitHub Releases -> 本地 latest.bin -> dry_run -> 真实 CAN OTA
```

## 6. 启动

```bash
ros2 launch stm32_can_ota stm32_can_ota.launch.py
```

默认启动后，节点会使用下面这个默认固件路径：

```text
$HOME/.stm32_can_ota/firmware/latest.bin
```

如果你想临时换一个默认固件路径，可以启动时覆盖：

```bash
ros2 launch stm32_can_ota stm32_can_ota.launch.py \
  default_firmware_path:=/home/lsz/.stm32_can_ota/firmware/slave_app.bin
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
| `default_firmware_path` | `$HOME/.stm32_can_ota/firmware/latest.bin` | service 未传路径时使用的默认固件 |
| `block_size` | `256` | OTA block 大小，后续应以 Bootloader capability 为准 |
| `ack_timeout_ms` | `1000` | ACK 超时 |
| `capability_timeout_ms` | `1000` | capability 查询超时 |
| `receive_timeout_ms` | `100` | CAN 接收轮询超时 |
| `max_retries` | `3` | 每个控制命令或块失败后的最大重试次数 |
| `inter_frame_delay_us` | `0` | DATA 帧间可选节流延时，默认由 SocketCAN 排队 |
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
string resolved_firmware_path
uint32 image_size
uint32 image_crc32
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
  "{firmware_path: '', firmware_version: 1, dry_run: true}"
```

正常响应示例：

```text
response:
stm32_can_ota.srv.StartOta_Response(
  accepted=True,
  state=3,
  error_code=0,
  message='dry run complete; firmware image loaded, no CAN frames sent',
  resolved_firmware_path='/home/lsz/.stm32_can_ota/firmware/latest.bin',
  image_size=37192,
  image_crc32=1452055584)
```

其中 `image_crc32` 是 ROS2 按 `uint32` 打印出的十进制数，用来确认 OTA 节点读取到的固件内容是否符合预期。

真实传输：

```bash
ros2 service call /stm32_ota/start stm32_can_ota/srv/StartOta \
  "{firmware_path: '', firmware_version: 1, dry_run: false}"
```

service 返回 `accepted=true` 表示后台任务已启动，最终结果应查看状态 topic。取消任务：

```bash
ros2 service call /stm32_ota/cancel stm32_can_ota/srv/CancelOta "{}"
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

- STM32 Bootloader 接收端尚未实现，当前通过 FakeTransport 单元测试验证完整主机流程
- 当前 `firmware_package` 仅支持内部清单结构体（`FirmwarePackageManifest`），尚未支持外部 `manifest.json` / YAML 解析
- 未实现固件签名、防降级、掉电恢复和 A/B 回滚
- 尚未在真实 CAN 总线和 STM32 Flash 上完成硬件联调

线协议、CRC 参数、重试语义和下位机最小实现要求见
[经典 CAN OTA 协议 v1](docs/protocol_v1.md)。

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
