# robmini_description（humble_real 分支）

`robmini_description` 是 `humble_real` 分支中的机器人描述与 bringup 核心包，用于把机器人模型、控制器配置和真机/仿真启动方式组织在一起，给实机部署提供统一入口。

---

## 1. 包定位

在实机分支中，这个包承担的职责非常集中：

- 定义机器人模型
- 组织控制器参数
- 生成机器人描述参数
- 启动 `ros2_control_node`
- 发布机器人状态与 TF
- 提供仿真和真机启动脚本

---

## 2. 目录结构

```text
robmini_description/
├── config/
│   ├── robmini_controllers.yaml
│   └── robmini_mecanum_controllers.yaml
├── launch/
│   ├── includes/
│   ├── display.launch.py
│   ├── real_bringup.launch.py
│   ├── sim_01_launch.py
│   ├── sim_02_launch.py
│   ├── simulation.launch.py
│   └── teleop.launch.py
├── meshes/
├── rviz/
├── urdf/
│   ├── lidar_gazebo.xacro
│   ├── robmini.ros2_control.xacro
│   ├── robmini.urdf.xacro
│   ├── robmini_run.urdf.xacro
│   └── ros2_control_sim.xacro
├── worlds/
├── CMakeLists.txt
└── package.xml
```

---

## 3. 核心文件说明

### `launch/real_bringup.launch.py`

该文件是真机 bringup 的核心入口，通常完成：

- 读取 `robmini_run.urdf.xacro`
- 通过 Xacro 生成 `robot_description`
- 加载 `robmini_mecanum_controllers.yaml`
- 启动 `controller_manager/ros2_control_node`
- 启动 `robot_state_publisher`
- 通过定时器延时加载广播器与麦克纳姆控制器

这是整车真机控制最重要的启动文件之一。

### `config/robmini_mecanum_controllers.yaml`

该文件定义麦克纳姆底盘控制器相关参数，是控制器能否正确加载的关键。

### `urdf/robmini_run.urdf.xacro`

通常用于真机运行场景，与纯仿真模型有所区分。

### `urdf/robmini.urdf.xacro`

通常作为机器人主模型骨架。

---

## 4. 典型启动方式

### 真机 bringup

```bash
ros2 launch robmini_description real_bringup.launch.py
```

### 仿真

```bash
ros2 launch robmini_description simulation.launch.py
```

### 模型显示

```bash
ros2 launch robmini_description display.launch.py
```

---

## 5. 真机 bringup 的职责拆解

在实机部署中，`real_bringup.launch.py` 主要做了三件事：

### 5.1 生成机器人描述

通过 Xacro 加载机器人模型，并允许使用参数控制命名空间或 mock 模式。

### 5.2 启动控制器管理器

启动 `ros2_control_node`，让硬件接口插件真正开始工作。

### 5.3 启动状态发布与控制器加载

- `robot_state_publisher` 发布机器人 TF
- 延时启动关节广播器
- 延时启动麦克纳姆控制器

这套顺序是典型的 ROS2 控制 bringup 结构。

---

## 6. 维护建议

### 保持关节命名统一

URDF、控制器 YAML、硬件接口代码三者一定要一致。

### 仿真与真机复用模型骨架

尽量在 Xacro 层复用通用部分，减少重复维护。

### 重要 launch 注释尽量保留

这个包的启动逻辑是多人协作时最容易出问题的地方，清晰注释很有价值。

---

## 7. 常见问题

### 控制器加载失败

优先检查：

- 插件库是否存在
- 参数文件路径是否正确
- URDF 是否包含对应接口
- 关节名是否匹配

### 模型显示正常但 TF 不完整

检查 `robot_state_publisher` 是否成功启动。

### 能启动但小车不动

问题通常不在模型层，而在控制器或底层串口硬件接口层。

---

## 8. 项目表述示例

> 完成麦克纳姆轮机器人 URDF/Xacro 建模与真机 bringup 设计，集成 `ros2_control` 控制器加载、机器人状态发布、仿真/实机双场景启动和 RViz/Gazebo 支持，为实车导航部署提供统一入口。