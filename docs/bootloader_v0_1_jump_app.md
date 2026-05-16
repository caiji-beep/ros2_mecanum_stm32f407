# Bootloader v0.1 跳转 APP_A 验证说明

本文用于验证 STM32F407VE Bootloader v0.1 是否能够从 `0x08000000` 启动，并在 APP_A 有效时跳转到 `0x08010000`。

## 1. Flash 分区表

| 分区 | 起始地址 | 结束地址 | 大小 | 说明 |
| --- | --- | --- | --- | --- |
| Bootloader | `0x08000000` | `0x0800BFFF` | 48KB | Bootloader 程序区 |
| Boot Param | `0x0800C000` | `0x0800FFFF` | 16KB | Boot 参数区，v0.1 暂不使用 |
| APP_A | `0x08010000` | `0x0803FFFF` | 192KB | 当前验证跳转目标 |
| APP_B | `0x08040000` | `0x0807FFFF` | 256KB | 预留备用 APP 区 |

## 2. Bootloader 工程 Memory Layout

Bootloader 工程应从 Flash 起始地址运行，只占用 Bootloader 分区。

| 配置项 | 值 |
| --- | --- |
| IROM1 Start | `0x08000000` |
| IROM1 Size | `0xC000` |
| IRAM1 Start | `0x20000000` |
| IRAM1 Size | `0x20000` |
| IROM2 | 不勾选 |
| IRAM2 | 不勾选 |

## 3. APP 工程 Memory Layout

APP 工程必须链接到 APP_A 起始地址，否则 Bootloader 跳转后向量表和函数地址会不匹配。

| 配置项 | 值 |
| --- | --- |
| IROM1 Start | `0x08010000` |
| IROM1 Size | `0x30000` |
| IRAM1 Start | `0x20000000` |
| IRAM1 Size | `0x20000` |
| IROM2 | 不勾选 |
| IRAM2 | 不勾选 |

## 4. APP 向量表偏移说明

APP_A 的向量表位于 `0x08010000`，因此 APP 启动后必须保证中断向量表指向 APP_A。

可使用以下任一方式：

```c
#define VECT_TAB_OFFSET  0x10000
```

或在 APP 初始化时显式设置：

```c
SCB->VTOR = 0x08010000;
```

如果 APP 链接到了 `0x08010000`，但 `SCB->VTOR` 仍指向 `0x08000000`，APP 主循环可能能运行，但中断会跳到 Bootloader 的向量表，导致中断异常。

## 5. 烧录顺序

推荐烧录顺序：

1. 先烧录 APP 到 `0x08010000`。
2. 再烧录 Bootloader 到 `0x08000000`。
3. 避免使用 Full Chip Erase。
4. 使用 Sector Erase，只擦除当前要下载的分区。

注意：

- 下载 APP 时不要擦掉 `0x08000000 ~ 0x0800BFFF` 的 Bootloader。
- 下载 Bootloader 时不要擦掉 `0x08010000 ~ 0x0803FFFF` 的 APP_A。
- 如果使用 `.hex` 文件，地址通常已包含在文件中。
- 如果使用 `.bin` 文件，必须手动指定下载基址。

## 6. 验证方法

烧录完成后，可先用调试器或烧录工具查看 APP_A 起始地址内容。

检查 `0x08010000`：

- 该地址保存 APP 初始 MSP。
- 正常应为 `0x200xxxxx` 范围内的 SRAM 地址。
- 常见值可能是 `0x20020000`。
- 如果为 `0xFFFFFFFF`，说明 APP_A 没有烧录成功，或被擦除了。

检查 `0x08010004`：

- 该地址保存 APP 的 Reset_Handler 地址。
- 正常应为 `0x0801xxxx`。
- Cortex-M4 运行 Thumb 指令，Reset_Handler 地址 bit0 应为 1，因此常见表现为奇数地址。

复位验证：

1. 复位 MCU。
2. MCU 从 `0x08000000` 启动 Bootloader。
3. Bootloader 检查 `0x08010000` 的 APP_A 向量表。
4. 如果 APP_A 有效，Bootloader 设置 `VTOR`、`MSP`，并跳转到 APP_A Reset_Handler。
5. 观察 APP_A 是否进入预期运行状态。

## 7. 常见问题

### APP 能跑但中断异常

原因通常是 APP 的向量表没有指向 `0x08010000`。

检查：

- `VECT_TAB_OFFSET` 是否为 `0x10000`。
- 或 APP 初始化后是否执行了 `SCB->VTOR = 0x08010000`。
- APP 工程是否确实链接到 `0x08010000`。

### `0x08010000` 是 `0xFFFFFFFF`

说明 APP_A 区为空或已被擦除。

检查：

- APP 是否已烧录到 `0x08010000`。
- 下载 APP 时是否选错基址。
- 后续下载 Bootloader 时是否执行了 Full Chip Erase。

### Bootloader 无法判断 APP 有效

Bootloader v0.1 判断 APP 有效的关键条件：

- `0x08010000` 的初始 MSP 必须在 SRAM 范围内。
- `0x08010004` 的 Reset_Handler 必须在 APP_A Flash 范围内。
- Reset_Handler 地址 bit0 必须为 1。
- APP_A 起始地址必须与 Bootloader 配置的 `APP_A_ADDR` 一致。

### 下载 APP 时擦掉了 Bootloader

如果下载 APP 后复位无法进入 Bootloader，可能是下载过程执行了 Full Chip Erase。

处理方式：

- 改用 Sector Erase。
- 重新烧录 APP 到 `0x08010000`。
- 再重新烧录 Bootloader 到 `0x08000000`。
- 烧录后再次确认 `0x08000000` 和 `0x08010000` 都不是 `0xFFFFFFFF`。
