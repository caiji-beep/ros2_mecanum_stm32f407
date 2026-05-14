# USER（stm32f407 分支）

`USER` 目录是该下位机工程的 **入口与工程组织中心**。如果说：

- `HARDWARE` 是驱动层
- `MyTasks` 是任务层
- `SYSTEM` 是公共支撑层

那么 `USER` 就是把这些内容真正“拼起来”的地方。

---

## 1. 目录结构

当前目录包含：

```text
USER/
├── DebugConfig/
├── JLinkSettings.ini
├── Template.uvguix.*
├── Template.uvoptx
├── freertos_demo.c / freertos_demo.h
├── main.c / main.h
├── mecanum.uvguix.*
└── （其余 Keil 工程相关文件）
```

---

## 2. 文件说明

### `main.c / main.h`

系统主入口。

一般负责：

- 芯片基础初始化
- 时钟与外设基础初始化
- FreeRTOS 启动前准备
- 最终进入调度器

这是整个下位机“从上电到运行”的第一入口。

### `freertos_demo.c / freertos_demo.h`

FreeRTOS 任务创建与系统启动组织文件。

通常负责：

- 创建各业务任务
- 分配任务优先级
- 分配栈大小
- 启动调度器

### `DebugConfig/`

调试配置目录。

### `JLinkSettings.ini`

J-Link 调试配置文件。

### `.uvoptx / .uvguix / .uvprojx` 类文件

Keil 工程与界面配置文件，用于直接在 MDK 中打开和调试工程。

---

## 3. 目录定位

`USER` 目录的价值在于：

- 给整个工程提供统一入口
- 保存调试工程配置
- 组织 RTOS 启动流程

如果要快速上手这个分支，通常就应该先从 `USER/main.c` 和 `USER/freertos_demo.c` 开始。

---

## 4. 推荐阅读顺序

### 第一步：看 `main.c`

搞清楚：

- 上电后先初始化了什么
- FreeRTOS 在哪里接管系统

### 第二步：看 `freertos_demo.c`

搞清楚：

- 创建了哪些任务
- 任务优先级如何分配
- 调度器什么时候启动

### 第三步：再去看 `MyTasks/`

带着全局视角看任务代码，更容易理解系统结构。

---

## 5. 工程维护建议

### 把“系统入口”和“业务逻辑”分开

不要在 `main.c` 里塞大量业务代码。

### RTOS 创建逻辑集中管理

任务创建与优先级建议统一放在一个文件里管理，便于整体把控。

### 保持工程文件整洁

Keil 生成的用户界面配置文件较多，建议 README 中注明哪些必须跟踪、哪些可忽略。

---

## 6. 适合写在项目介绍里的表述

> 负责 STM32F407 下位机工程入口与 FreeRTOS 启动组织，完成系统初始化、任务创建、调试配置与 Keil 工程管理，为机器人底层多任务控制系统提供统一工程框架。