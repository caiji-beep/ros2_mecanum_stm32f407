# STM32 经典 CAN OTA 协议 v1

本文档定义 `stm32_can_ota` 上位机当前实现的线协议。下位机尚未实现时，应以本文档作为
Bootloader 接收端的实现依据。

## 1. 基本约定

- 使用经典 CAN 2.0，一帧最多 8 字节，不依赖 CAN FD。
- 默认请求 ID 为 `0x600 + node_id`，应答 ID 为 `0x580 + node_id`。
- 所有多字节整数均为小端序。
- 控制帧 DLC 固定为 8；DATA 帧 DLC 为 `3 + 有效数据长度`。
- 一次升级由非零 16 位 `session_id` 标识。
- v1 最大块大小为 1280 字节，因为一个块最多有 256 个 DATA sequence，每帧 5 字节。
- 默认块大小为 256 字节，每块需要 52 个 DATA 帧。

## 2. 命令码

| 命令 | 值 | 方向 | 作用 |
| --- | --- | --- | --- |
| `SYNC` | `0x01` | Host → Bootloader | 确认 Bootloader 在线 |
| `GET_CAPABILITY` | `0x02` | Host → Bootloader | 分页查询能力 |
| `START` | `0x10` | Host → Bootloader | 发送 session 和镜像大小 |
| `START_CRC` | `0x11` | Host → Bootloader | 发送完整镜像 CRC32 |
| `START_VERSION` | `0x12` | Host → Bootloader | 发送完整 32 位版本号 |
| `BLOCK_BEGIN` | `0x20` | Host → Bootloader | 开始或重新开始一个块 |
| `DATA` | `0x21` | Host → Bootloader | 携带最多 5 字节镜像数据 |
| `BLOCK_END` | `0x22` | Host → Bootloader | 提交块及 CRC16 |
| `END` | `0x30` | Host → Bootloader | 请求 Flash 整包 CRC32 校验 |
| `ABORT` | `0x31` | Host → Bootloader | 取消当前 session |
| `ACK` | `0x79` | Bootloader → Host | 命令成功 |
| `NACK` | `0x1f` | Bootloader → Host | 命令失败 |

## 3. 请求帧布局

> 约定：所有多字节整数均为**小端**（低字节在前）。下方格子图下标 `b0~b7` 对应 CAN 帧
> `data[0..7]`。每个格子标注"值(含义)"，`u16`/`u32` 跨两/四格且低字节在左。

### 3.1 START 系列（元数据，三帧）

元数据拆成三帧，避免经典 CAN 的 8 字节载荷截断 CRC32 或版本号。

```text
START:         [10, session:u16, image_size:u32, reserved]
START_CRC:     [11, session:u16, image_crc32:u32, reserved]
START_VERSION: [12, session:u16, firmware_version:u32, reserved]
```

**START 帧格子图（`0x10`）：**

```text
b:    0    1          2          3    4          5          6          7
     ┌────┬──────────────────────┬────┬──────────────────────────────────────┐
     │ 10 │      session         │ rs │           image_size                │
     │cmd │      (u16,小端)      │ rv │           (u32,小端)                │
     └────┴──────────────────────┴────┴──────────────────────────────────────┘
            b1=低字节 b2=高字节         b4..7 = 固件总字节数(低→高)
```

**START_CRC 帧（`0x11`）：** 与 START 同构，仅 `b0=0x11`，`b4..7 = image_crc32`(u32)。
**START_VERSION 帧（`0x12`）：** 同构，仅 `b0=0x12`，`b4..7 = firmware_version`(u32)。

实例（session=`0x0001`，image_size=`37192`=0x00009148）：

```text
START:        [10, 01, 00, 00, 48, 91, 00, 00]
                    ▲session=1   ▲image_size 低字节 48, 91, 00, 00 → 0x00009148 = 37192
```

### 3.2 BLOCK_BEGIN（`0x20`）

```text
[20, session:u16, block_index:u16, block_length:u16, reserved]
```

收到新的 `BLOCK_BEGIN` 时，下位机应清空该块的暂存区和 sequence 状态。因此主机可以在
超时或 NACK 后安全地重传整个块。

**格子图：**

```text
b:    0    1          2          3          4          5    6    7
     ┌────┬──────────────────────┬──────────────────────┬──────────────────────┬────┐
     │ 20 │      session         │     block_index      │     block_length     │ rs │
     │cmd │      (u16)           │      (u16,小端)       │      (u16,小端)       │ rv │
     └────┴──────────────────────┴──────────────────────┴──────────────────────┴────┘
```

实例（session=1，第 0 块，块长 256）：

```text
BLOCK_BEGIN: [20, 01, 00, 00, 00, 00, 00, 01]
                   ▲sess=1   ▲block_index=0   ▲block_length 00 01 → 256
```

### 3.3 DATA（`0x21`）

```text
[21, sequence:u8, payload_length:u8, payload:0..5 bytes]
```

- sequence 在每个块内从 0 重新开始。
- `payload_length` 范围为 1～5。
- DLC 必须等于 `3 + payload_length`，最后一帧不依赖补零。
- 下位机应拒绝跳号、重复或超过 `BLOCK_BEGIN.block_length` 的数据。

**格子图：**

```text
b:    0    1          2          3          4          5          6          7
     ┌────┬──────────┬──────────┬──────────┬──────────┬──────────┬──────────┬────┐
     │ 21 │ sequence │ payload_ │      payload (最多 5 字节，小端按字节序原样)      │
     │cmd │  (u8)    │  length  │   b3 ... b7 仅前 payload_length 字节有效          │
     └────┴──────────┴──────────┴──────────┴──────────┴──────────┴──────────┴────┘
```

实例（块内第 0 帧，载荷 5 字节 `DE AD BE EF A5`）：

```text
DATA: [21, 00, 05, DE, AD, BE, EF, A5]
           ▲seq=0  ▲len=5   ▲5 字节固件数据
```

> 注意：一帧只能装 5 字节，256 字节的块需 52 帧 DATA。最后一帧 `payload_length` 可能 < 5。

### 3.4 BLOCK_END（`0x22`）

```text
[22, session:u16, block_index:u16, block_crc16:u16, reserved]
```

CRC16 使用 CRC-16/CCITT-FALSE：多项式 `0x1021`、初值 `0xFFFF`。下位机只有在长度、
sequence 和 CRC16 全部正确后才写入 Flash 并返回 ACK。

**格子图：**

```text
b:    0    1          2          3          4          5          6          7
     ┌────┬──────────────────────┬──────────────────────┬──────────────────────┬────┐
     │ 22 │      session         │     block_index      │     block_crc16      │ rs │
     │cmd │      (u16)           │      (u16,小端)       │      (u16,小端)       │ rv │
     └────┴──────────────────────┴──────────────────────┴──────────────────────┴────┘
```

实例（session=1，块 0，块 CRC16=`0x1A2B`）：

```text
BLOCK_END: [22, 01, 00, 00, 00, 2B, 1A]
                  ▲sess=1  ▲blk=0  ▲crc16 低字节 2B, 高字节 1A → 0x1A2B
```

### 3.5 END 与 ABORT

```text
END:   [30, session:u16, image_crc32:u32, reserved]
ABORT: [31, session:u16, 00, 00, 00, 00, 00]
```

CRC32 使用 IEEE CRC-32：反射多项式 `0xEDB88320`、初值 `0xFFFFFFFF`、最终取反。
END 阶段下位机应从目标 Flash 区重新计算 CRC32，而不是只校验 RAM 中的数据。

**END 帧格子图（`0x30`）：**

```text
b:    0    1          2          3    4          5          6          7
     ┌────┬──────────────────────┬────┬──────────────────────────────────────┐
     │ 30 │      session         │ rs │           image_crc32               │
     │cmd │      (u16)           │ rv │           (u32,小端)                │
     └────┴──────────────────────┴────┴──────────────────────────────────────┘
```

**ABORT 帧格子图（`0x31`）：** `b0=0x31`，`b1..2=session(u16)`，其余 5 字节全 `0x00`。

实例（session=1）：

```text
END:   [30, 01, 00, 00, C0, DE, A5, 01]   ← image_crc32 = 0x01A5DEC0 (示例)
ABORT: [31, 01, 00, 00, 00, 00, 00, 00]
```

## 4. Capability 分页应答

主机依次请求 page 0 和 page 1：

```text
请求: [02, page, 00, 00, 00, 00, 00, 00]

page 0 ACK (DLC=8):
[79, 02, status=00, page=00, protocol_version:u8, target_slot:u8, block_size:u16(6..7)]

page 1 ACK (DLC=8):
[79, 02, status=00, page=01, max_fw_size:u32(4..7)]
```

字段字节偏移（以 0 为帧起始）：

- 第 0 字节：`0x79` (ACK)
- 第 1 字节：回显命令 `0x02` (GET_CAPABILITY)
- 第 2 字节：`status`，必须为 `0` 表示成功
- 第 3 字节：回显 `page`（0 或 1）
- page 0：
  - 第 4 字节：`protocol_version`（u8）
  - 第 5 字节：`target_slot`（u8）
  - 第 6..7 字节：`block_size`（u16，小端）
- page 1：
  - 第 4..7 字节：`max_fw_size`（u32，小端）

注意：两页的应答里**没有** `session:u16` 字段（GET_CAPABILITY 发生在会话建立之前，
此时 session_id 尚为 0）。`status=0` 表示成功。字符串、分区布局和 Flash 页细节不放在
线协议中；它们属于下位机内部实现或未来 capability 扩展页。

### 4.1 逐字节布局图（对齐代码 `parseCapability`）

下面用"格子图"展示一帧 8 字节里每个格子装什么。下标 `byte0~byte7` 是 CAN 帧 `data[0..7]`。

**请求（Host → Bootloader，两页都用同一格式，只改 byte1 的 page）：**

```text
byte:   0    1    2    3    4    5    6    7
      ┌────┬────┬────┬────┬────┬────┬────┬────┐
      │ 02 │page│ 00 │ 00 │ 00 │ 00 │ 00 │ 00 │
      └────┴────┴────┴────┴────┴────┴────┴────┘
                 ▲ page=0 查第一页；page=1 查第二页
```

**应答 page 0（Bootloader → Host）：**

```text
byte:   0    1    2    3    4          5          6          7
      ┌────┬────┬────┬────┬─────────┬─────────┬──────────────────────┐
      │ 79 │ 02 │ 00 │ 00 │proto_ver│target_  │     block_size       │
      │ACK │cmd │stat│page│  (u8)   │ slot(u8)│      (u16, 小端)      │
      └────┴────┴────┴────┴─────────┴─────────┴──────────────────────┘
                                                    ▲ byte6=低字节
                                                      byte7=高字节
```

- `byte4 = protocol_version`：协议版本号（u8，当前固定 `1`）
- `byte5 = target_slot`：建议烧写的镜像槽（u8）
- `byte6..7 = block_size`：单块字节数（u16 小端，例如 `0x00 0x01` 表示 256）

**应答 page 1（Bootloader → Host）：**

```text
byte:   0    1    2    3    4          5          6          7
      ┌────┬────┬────┬────┬──────────────────────────────────────┤
      │ 79 │ 02 │ 00 │ 01 │            max_fw_size               │
      │ACK │cmd │stat│page│            (u32, 小端)               │
      └────┴────┴────┴────┴──────────────────────────────────────┘
                         ▲ byte4=最低字节 … byte7=最高字节
```

- `byte4..7 = max_fw_size`：允许烧写的最大固件字节数（u32 小端）

> 小端（little-endian）含义：多字节整数**低字节在前**。例如 `block_size = 256`，
> 写成十六进制 `0x0100`，传输时 `byte6 = 0x00`、`byte7 = 0x01`（低字节 0x00 在前）。

### 4.2 完整交互实例

假设 Bootloader 能力为：协议版本 `1`、目标槽 `0`、块大小 `256`、最大固件 `0x00020000`(131072)。

```text
Host  → [02, 00, 00, 00, 00, 00, 00, 00]    请求 page 0
Boot  ← [79, 02, 00, 00, 01, 00, 00, 01]    协议版本=1, target_slot=0, block_size=0x0100=256
             byte4=1  byte5=0  byte6=0x00 byte7=0x01

Host  → [02, 01, 00, 00, 00, 00, 00, 00]    请求 page 1
Boot  ← [79, 02, 00, 01, 00, 02, 00, 00]    max_fw_size=0x00020000=131072
             byte4..7 = 00 02 00 00  → 小端拼成 0x00020000
```

上位机 `OtaClient` 拿到后会：
1. 校验 `protocol_version == 1`；
2. 校验 `block_size` 在 `1..1280` 之间（受 DATA 帧每帧 5 字节、最多 256 个 sequence 限制）；
3. 若 `max_fw_size != 0` 且固件超过它，则报 `firmware_too_large` 中止。

下位机实现时**必须严格按上述字节偏移填充**，否则 `parseCapability` 解析出的字段会错位
（例如把 `block_size` 误读成 `target_slot` 的值），导致升级流程异常或块大小判断失败。

## 5. 通用 ACK/NACK

除 capability 外，控制命令使用统一应答：

```text
[79/1F, echoed_command, status, session:u16, detail:u16, next_sequence:u8]
```

- ACK 必须使用 `0x79` 且 `status=0`。
- NACK 使用 `0x1F`，或 ACK 标记但带非零 status；主机都会视为失败。
- `BLOCK_BEGIN/BLOCK_END` 的 detail 应返回 block index。
- 数据缺失时，NACK 的 `next_sequence` 可返回期望的 sequence，便于诊断。

建议下位机状态码：`1=bad_state`、`2=bad_session`、`3=block_crc`、
`4=sequence_error`、`5=image_crc`、`6=size_error`、`7=flash_error`、
`8=version_rejected`。

## 6. 主机状态机和重试

```text
load image → SYNC → capability page 0/1
           → START → START_CRC → START_VERSION
           → (BLOCK_BEGIN → DATA×N → BLOCK_END)×block_count
           → END → completed
```

- SYNC、capability、START 系列和 END 在超时/NACK 后重发控制命令。
- 块内任意发送错误、BLOCK_END 超时或 NACK 都会从 BLOCK_BEGIN 重传整个块。
- 默认允许 3 次重试，即最多发送 4 次。
- DATA 不逐帧等待 ACK，避免总线吞吐量被往返应答严重降低。
- CAN 控制器自身负责位错误检测和链路层自动重发；块 CRC 和整包 CRC 负责应用层完整性。
- 取消时主机发送一次 ABORT，并终止后台任务。

## 7. 下位机最小实现要求

下位机至少需要：Bootloader 入口机制、协议状态机、块 RAM 缓冲、Flash 擦写、CRC16、
Flash 整包 CRC32、ACK/NACK、升级成功标志和重启。若要用于真实产品，还应增加固件签名、
防回滚、掉电恢复和 A/B 分区。
