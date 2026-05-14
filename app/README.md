# ros2_mecanum_stm32f407（stm32f407 分支）

`stm32f407` 分支是整个项目的 **下位机固件主线**，面向真实麦克纳姆轮底盘控制板，基于 **STM32F407 + FreeRTOS** 实现多任务调度、通信、传感器接入、状态管理与基础执行控制。

---

## 1. 分支定位

这个分支的角色不是仿真，也不是 ROS2 上位机，而是：

- 接收上位机或其他控制板发来的控制命令
- 驱动底盘电机与执行机构
- 读取编码器与 IMU 等传感器
- 管理显示、蜂鸣器、看门狗等系统功能
- 在实时系统下组织多任务运行

它是整台机器人“真正落地在硬件上的控制核心”。

---

## 2. 目录结构

```text
stm32f407/
├── CORE/
├── EIDE/
├── FWLIB/
├── FreeRTOS/
├── HARDWARE/
├── MyTasks/
├── OBJ/
├── SYSTEM/
├── USER/
├── README.md
└── keilkilll.bat
```

---

## 3. 关键目录说明

### `CORE/`

通常存放芯片启动、核心中断或基础初始化相关内容。

### `FWLIB/`

STM32 标准外设库或工程依赖库目录。

### `FreeRTOS/`

RTOS 内核、移植层与相关头文件。

### `HARDWARE/`

硬件驱动层，包括：

- CAN
- 编码器
- IMU
- OLED
- PWM
- 蜂鸣器等

### `MyTasks/`

任务层逻辑，是 FreeRTOS 应用层的核心。

### `SYSTEM/`

系统支撑模块，包括：

- 延时
- 串口重定向
- 定时器
- 机器人状态
- 看门狗等

### `USER/`

工程入口与 Keil 相关项目文件，通常包括：

- `main.c`
- `main.h`
- `freertos_demo.c`
- `.uvprojx/.uvoptx` 等工程文件

---

## 4. 当前工程特点

从仓库结构和 README 更新记录看，该分支已经包含以下能力方向：

- FreeRTOS 移植
- 状态机管理
- 死区补偿优化
- 软件看门狗
- 硬件看门狗
- 蜂鸣器提醒
- CAN 通信
- ICM20948 接入
- OLED 显示与翻页

这说明工程已经从“裸机驱动堆砌”进入了“系统化下位机软件”的阶段。

---

## 5. 适用场景

该分支适合用于：

- 底盘控制板固件开发
- 上下位机通信联调
- 编码器与 IMU 数据采集
- FreeRTOS 多任务系统实践
- 嵌入式机器人项目展示

---

## 6. 推荐开发环境

建议环境：

- STM32F407 硬件平台
- Keil MDK
- J-Link / ST-Link
- 串口调试工具
- CAN 调试工具

如果使用 EIDE，则也可结合 `EIDE/` 目录中的工程组织方式开发。

---

## 7. 推荐阅读顺序

如果第一次接手该分支，建议按以下顺序阅读：

1. `USER/main.c`：看系统入口
2. `USER/freertos_demo.c`：看任务创建方式
3. `MyTasks/`：看业务任务划分
4. `HARDWARE/`：看外设驱动能力
5. `SYSTEM/`：看系统支撑模块

这样能最快理解这个工程“从启动到业务运行”的完整链路。

---

## 8. 与 ROS2 分支的关系

该分支与 `humble` / `humble_real` 分支是上下位机关系：

- ROS2 分支负责导航、规划、上位机控制
- `stm32f407` 分支负责执行、采集和实时控制

两者之间通过串口或 CAN 等方式通信。

---

## 9. 适合写在项目介绍里的表述

> 基于 STM32F407 与 FreeRTOS 构建麦克纳姆轮机器人下位机固件，完成多任务调度、状态机管理、编码器/IMU 数据采集、CAN/串口通信、OLED 显示、看门狗与蜂鸣器等系统功能，为 ROS2 上位机导航系统提供底层执行与反馈支持。



<!--
 * @Author: caiji-beep 2978115384@qq.com
 * @Date: 2025-12-01 19:02:11
 * @LastEditors: caiji-beep 2978115384@qq.com
 * @LastEditTime: 2026-04-06 22:14:35
 * @FilePath: \ros2_mecanum\README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
  git config --global user.email "you@example.com"
  git config --global user.name "Your Name"

备份
更新1（截止2025.11.22 20:34）：
1.移植RTOS
2.添加状态机管理
更新2（2025112301）
1.添加死区补偿及优化
2.软件看门狗（Nav command watchdog）
3.一些bug
更新3（2025112302）
1.硬件看门狗
更新4（2025112303）
1.添加蜂鸣器提醒
2.一些bug
3.添加icm20948驱动（待移植进系统）


更新5（2025120301）
1.移植了can通信（与stm32f103）
2.将icm20948驱动移植入系统（硬件IIC）
3.解决oled屏幕卡死bug
4.初步解决导航时can发送的imu数据不更新
5.待解决问题
  5.1 按键翻页显示问题
  5.2 与上位机can通信
  5.3 ICM20948改成软件IIC（视情况而定） 

更新6（2025120401）
1.按键翻页显示问题
2.与上位机can通信