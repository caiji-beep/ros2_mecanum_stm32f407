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

### 6.4 仿真多机器人导航

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_navigation.launch.py
```

默认会生成 `robmini_01` 和 `robmini_02` 两台车；RViz2 直接加载包内 `multi_robots_navigation.rviz`。
使用工具栏里的 `2D Goal Pose` 下发导航目标，默认 Topic 是 `/robmini_01/goal_pose`；要指挥 2 号车，在 Tool Properties 里把 Topic 改成 `/robmini_02/goal_pose`。
这里改的是 `2D Goal Pose` 的 Topic，不是 `Publish Point` 的 `clicked_point`；`clicked_point` 只发布点坐标，不能触发 Nav2 导航。

使用自定义地图时传入相对 `maps/` 的路径：

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_navigation.launch.py \
  map_file:=merged_multi/merged_map_01.yaml
```

注意地图文件名要和实际文件一致，例如 `merged_map_01.yaml` 不是 `merger_map_01.yaml`。局部测试地图可以加载，但机器人初始位姿和目标点必须落在已建图区域；如果 AMCL 没有发布 `map -> odom`，先确认 `/robmini_01/map` 有数据，再用 RViz 的 `2D Pose Estimate` 给对应机器人重新设初始位姿。
如果刚保存的新地图放进 `src/robmini_navigation/maps/` 后 launch 仍找不到，重新执行 `colcon build --symlink-install` 并 `source install/setup.bash`。

### 6.5 仿真多机器人共同建图

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_mapping.launch.py
```

默认启动 `robmini_01` 和 `robmini_02` 两台仿真车，每台车各自运行一套 `slam_toolbox`。
合图节点会订阅 `/robmini_01/map` 与 `/robmini_02/map`，并发布合成后的 `/merged_map`；RViz2 会同时显示总图和每台车的局部建图结果。

建图时用 teleop 分别控制两台车：

```bash
ros2 launch robmini_navigation teleop.launch.py \
  robot_name:=robmini_01 \
  namespace:=robmini_01
```

```bash
ros2 launch robmini_navigation teleop.launch.py \
  robot_name:=robmini_02 \
  namespace:=robmini_02
```

共同建图时两台车会互相进入雷达范围，单车自己的 `/robmini_01/map`、`/robmini_02/map` 里可能出现对方车身留下的小黑点。当前方案不再过滤送入 `slam_toolbox` 的 `/scan`，也不再提供 `filter_peer_robots` 参数，避免滤波节点和 SLAM 在启动初期互相等待 TF；而是在 `/merged_map` 合图输出前，根据两台车走过的轨迹清理动态车身残影。这个清理只作用于融合图，不会反向改写每台车自己的 SLAM 地图。保存地图时请保存 `/merged_map`，不要保存单车 map。

如 `/merged_map` 上仍有残点，可适当增大清理半径：

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_mapping.launch.py \
  clear_robot_trail_radius:=0.45 \
  clear_start_radius:=0.55
```

保存合并地图：

```bash
mkdir -p /home/lsz/robotmini_ws/src/robmini_navigation/maps/merged_multi

ros2 run nav2_map_server map_saver_cli \
  -t /merged_map \
  -f /home/lsz/robotmini_ws/src/robmini_navigation/maps/merged_multi/merged_map_01 \
  --fmt pgm
```

保存后会得到 `merged_map_01.yaml` 和 `merged_map_01.pgm`，多机导航时用 `map_file:=merged_multi/merged_map_01.yaml` 加载。

如果两张子地图在 RViz2 中有偏移，可以通过 `map_origin_x_1/map_origin_y_1/map_origin_yaw_1` 和 `map_origin_x_2/map_origin_y_2/map_origin_yaw_2` 调整每台车的 `map` 到 `world` 的静态对齐关系。

### 6.6 键盘控制

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

快速启动（ubantu远程ssh）：
1.ros2 launch robmini_navigation demo_real_navigation.launch.py 
2.新建远程终端
3.ros2 run rviz2 rviz2 -d /home/lsz/robotmini_ws/install/robmini_navigation/share/robmini_navigation/rviz/nav2.rviz --ros-args -r __ns:=/robmini -p use_sim_time:=false
4.蓝牙将小车改为nav模式
202605021927
