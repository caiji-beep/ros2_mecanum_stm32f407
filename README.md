# ros2_mecanum_stm32f407（humble_real 分支）

`humble_real` 分支面向 **ROS2 实机联调与落地运行**。如果说 `humble` 更偏向“开发工作区”，那么 `humble_real` 更偏向“实机部署结构”：ROS2 包直接位于仓库根目录，启动方式更直接，适合接入真实底盘、激光雷达和 STM32 下位机。

---

## 1. 分支定位

该分支主要用于：

- 实际底盘 bringup
- 串口接入 STM32F407
- 轮速反馈接入 `ros2_control`
- 激光雷达接入
- 真机建图与导航
- 键盘控制实车运动

---

## 2. 目录结构

```text
humble_real/
├── can_socket_demo/
├── mecanum_hw_interface/
├── robmini_description/
├── robmini_navigation/
└── README.md
```

与 `humble` 分支相比，这个分支不再采用 `src/` 工作区二级目录，而是把各 ROS2 包直接放在仓库根目录，便于部署和使用。

---

## 3. 分支特点

### 特点 1：更偏真机

这个分支保留了：

- `real_bringup.launch.py`
- 真机构图 launch
- 真机导航 bringup
- 键盘控制节点
- 串口硬件接口

### 特点 2：更适合“直接跑包”

仓库结构更扁平，适合：

- 拷到设备后直接编译
- 多包一起部署
- 按功能模块独立维护 README

### 特点 3：保留调试补充包

虽然它以真机为主，但仍保留了 `can_socket_demo`，便于硬件联调时做底层 CAN 测试。

---

## 4. 推荐环境

建议环境：

- Ubuntu 22.04
- ROS2 Humble
- colcon
- rviz2
- nav2
- slam_toolbox
- ros2_control
- xacro
- 串口访问权限已配置

---

## 5. 编译

在该分支仓库根目录执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 6. 常用启动方式

### 6.1 真机 bringup

```bash
ros2 launch robmini_description real_bringup.launch.py
```

### 6.2 真机构图

```bash
ros2 launch robmini_navigation bringup_mapping_real.launch.py robot_name:=robmini
```

### 6.3 真机导航

```bash
ros2 launch robmini_navigation robot_bringup.launch.py \
  is_sim:=false \
  robot_name:=robmini \
  map_file:=room_mini/room_mini.yaml
```

### 6.4 键盘控制

```bash
ros2 run robmini_navigation mecanum_teleop_keyboard
```

---

## 7. 目录职责

| 目录                   | 作用                        |
| ---------------------- | --------------------------- |
| `can_socket_demo`      | CAN 接口监听示例            |
| `mecanum_hw_interface` | `ros2_control` 串口硬件接口 |
| `robmini_description`  | 模型、控制器、真机/仿真启动 |
| `robmini_navigation`   | 激光、建图、导航、键盘控制  |

---

## 8. 适合的使用顺序

建议实机联调时按下面顺序进行：

1. 先启动 `robmini_description`，确认 `ros2_control` 与 TF 正常
2. 再确认 `mecanum_hw_interface` 已和 STM32 串口接通
3. 再启动激光与建图流程
4. 最后进入地图定位和导航

如果一开始就直接上导航，往往很难定位问题是在底盘层、传感器层还是导航层。

---

## 9. 适合写在项目介绍里的表述

> 面向实机部署构建 ROS2 Humble 麦克纳姆轮机器人系统，完成真机 bringup、STM32 串口硬件接口接入、激光雷达建图、地图定位、Nav2 导航以及键盘遥控联调，形成仿真到实车的完整落地链路。

python3 ~/robotmini_ws/scripts/stm32_dummy_feeder.py --port /tmp/ttyV1 --baud 115200 --rate 30

ros2 launch robmini_description real_bringup.launch.py


ros2 run rviz2 rviz2 -d /home/lsz/robotmini_ws/install/robmini_navigation/share/robmini_navigation/rviz/nav2.rviz --ros-args -r __ns:=/robmini -p use_sim_time:=false

