# robmini_navigation（humble分支）

`robmini_navigation` 是 `humble_real` 分支中的导航与交互核心包，负责真机环境下的：

- 激光接入
- SLAM 建图
- Nav2 导航
- 地图管理
- 键盘控制

它是把机器人从“可控底盘”提升为“可导航移动机器人”的关键包。

---

## 1. 目录结构

```text
robmini_navigation/
├── config/
│   ├── nav2_params.yaml
│   ├── slam_template.yaml
│   └── teleop_params.yaml
├── launch/
│   ├── demo/
│   ├── bringup_mapping_real.launch.py
│   ├── bringup_mapping_sim.launch.py
│   ├── lidar.launch.py
│   ├── navigation_launch.py
│   ├── robot_bringup.launch.py
│   └── test1.launch.py
├── maps/
│   ├── merged_multi/
│   └── room_mini/
├── rviz/
├── scripts/
│   └── simple_map_merge.py
├── src/
│   ├── cycle.cpp
│   └── mecanum_teleop_keyboard.cpp
├── CMakeLists.txt
└── package.xml
```

---

## 2. 核心功能

### 2.1 真机构图

通过 `bringup_mapping_real.launch.py` 启动 SLAM 建图流程。

### 2.2 仿真建图

通过 `bringup_mapping_sim.launch.py` 在仿真中复用建图流程。

### 2.3 激光接入

通过 `lidar.launch.py` 启动激光雷达相关节点或配置。

### 2.4 定位导航总入口

通过 `robot_bringup.launch.py` 启动地图、AMCL、生命周期管理器和 Nav2 主导航逻辑。

### 2.5 键盘控制

通过 `mecanum_teleop_keyboard.cpp` 提供人工控制入口，适合真机联调。

---

## 3. 键盘控制节点说明

`mecanum_teleop_keyboard` 是一个非常实用的真机调试节点。它适合在导航之前先做底盘运动验证。

### 默认控制键位

- `w`：前进
- `s`：后退
- `a`：左移
- `d`：右移
- `q`：左转
- `e`：右转
- `x`：停止

### 默认速度参数

- `linear_speed = 0.2`
- `strafe_speed = 0.2`
- `angular_speed = 1.0`

如果底盘速度方向不符合预期，优先用这个节点做最小联调，而不是直接上导航。

---

## 4. 启动方式

### 真机构图

```bash
ros2 launch robmini_navigation bringup_mapping_real.launch.py robot_name:=robmini
```

### 仿真建图

```bash
ros2 launch robmini_navigation bringup_mapping_sim.launch.py robot_name:=robmini
```

### 仿真多机器人共同建图

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_mapping.launch.py
```

默认启动 `robmini_01` 与 `robmini_02` 两台仿真车，每台车各自运行独立 namespace 下的 `slam_toolbox`。
合图节点订阅 `/robmini_01/map` 和 `/robmini_02/map`，输出总图 `/merged_map`，RViz2 会直接加载包内 `multi_robots_mapping.rviz`。

建图时分别开终端控制两台车：

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

共同建图默认直接使用原始 `/scan`。之前尝试过在送入 `slam_toolbox` 前过滤同伴机器人激光点，但滤波节点需要 TF，SLAM 又需要 scan 才能稳定发布 TF，容易在启动阶段形成等待关系，所以当前已经移除 `filter_peer_robots` 相关参数和滤波脚本。

两车互相进入雷达范围时，单车自己的 `/robmini_01/map`、`/robmini_02/map` 里仍可能出现对方车身留下的小黑点。当前处理方式是在 `simple_map_merge.py` 发布 `/merged_map` 前，按两台车的启动区域和运动轨迹清理动态车身残影；这个清理只作用于融合图，不会反向改写每台车自己的 SLAM 地图。保存地图时请保存 `/merged_map`，不要保存单车 map。

如 `/merged_map` 上仍有残点，可适当增大清理半径：

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_mapping.launch.py \
  clear_robot_trail_radius:=0.45 \
  clear_start_radius:=0.55
```

这套流程适合仿真验证多机器人共同建图的 topic、TF 和 RViz 展示链路；它不是后端联合优化式 SLAM，地图对齐主要由每台车的 `map` 到 `world` 静态 TF 决定。
如需手动调整合图对齐，可传入 `map_origin_x_1/map_origin_y_1/map_origin_yaw_1` 或 `map_origin_x_2/map_origin_y_2/map_origin_yaw_2`。

保存融合地图：

```bash
mkdir -p /home/lsz/robotmini_ws/src/robmini_navigation/maps/merged_multi

ros2 run nav2_map_server map_saver_cli \
  -t /merged_map \
  -f /home/lsz/robotmini_ws/src/robmini_navigation/maps/merged_multi/merged_map_01 \
  --fmt pgm
```

保存后会得到 `merged_map_01.yaml` 和 `merged_map_01.pgm`。如果新地图放进包内后 launch 找不到，重新执行 `colcon build --symlink-install` 并 `source install/setup.bash`。

### 仿真多机器人导航

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_navigation.launch.py
```

默认启动 `robmini_01` 与 `robmini_02` 两台仿真车，每台车都有独立的 Nav2 namespace。
RViz2 直接加载包内 `multi_robots_navigation.rviz`，使用工具栏里的 `2D Goal Pose` 下发导航目标。
默认 Topic 是 `/robmini_01/goal_pose`；要指挥 2 号车，在 Tool Properties 里把 Topic 改成 `/robmini_02/goal_pose`。
这里改的是 `2D Goal Pose` 的 Topic，不是 `Publish Point` 的 `clicked_point`；`clicked_point` 只发布点坐标，不能触发 Nav2 导航。

加载共同建图保存的融合地图：

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_navigation.launch.py \
  map_file:=merged_multi/merged_map_01.yaml
```

地图路径相对 `robmini_navigation/maps/`。局部测试地图可以加载，但机器人初始位姿和目标点必须落在已建图区域；如果刚启动时 TF 里缺 `map -> odom`，通常是 AMCL 还没有定位成功，先确认对应 `/robmini_XX/map` 有数据，再在 RViz 用 `2D Pose Estimate` 给对应机器人重新设初始位姿。

### 真机导航

```bash
ros2 launch robmini_navigation robot_bringup.launch.py \
  is_sim:=false \
  robot_name:=robmini \
  map_file:=room_mini/room_mini.yaml
```

### 键盘控制

```bash
ros2 run robmini_navigation mecanum_teleop_keyboard
```

---

## 5. 配置文件说明

### `config/nav2_params.yaml`

Nav2 主参数配置。

### `config/slam_template.yaml`

SLAM 配置模板，可按 `robot_name` 做字符串替换，适合命名空间管理。

### `config/teleop_params.yaml`

键盘控制节点相关参数配置。

### `scripts/simple_map_merge.py`

仿真多机器人共同建图使用的轻量合图节点，订阅多台机器人的 `nav_msgs/OccupancyGrid`，根据 TF 转到统一坐标系后发布 `/merged_map`。
节点会在融合图输出前清理配置的启动区域和机器人轨迹区域，用来消除两车互相进入雷达范围时留下的动态车身残影。

---

## 6. 推荐实机联调顺序

1. 启动 `robmini_description`，确认 TF 和底盘 bringup 正常
2. 启动 `mecanum_teleop_keyboard`，确认前进/横移/旋转都正常
3. 启动 `lidar.launch.py`，确认雷达数据正常
4. 启动建图 launch，完成地图构建
5. 加载地图并启动导航

这个顺序的好处是：每一步都能单独验证，不会把问题叠在一起。

---

## 7. 常见问题

### 能键盘控制，但导航失败

说明底盘控制链路大概率正常，重点检查：

- 雷达数据
- TF
- 地图
- Nav2 参数

### 建图正常，但定位飘

重点检查：

- 轮速反馈精度
- `odom` 质量
- 激光外参
- 地图质量

### 仿真正常，实机不正常

不要先怀疑算法，先确认实机底层输入输出是否与仿真一致。

### 多机共同建图后导航一开始规划失败

优先看机器人初始位置是否落在融合地图里的静态障碍或未知区域。共同建图保存地图时应保存 `/merged_map`；如果保存了单车 map，或融合图上的车身残影没有清理干净，Nav2 刚启动时可能认为机器人在障碍区里。

### 多机导航 RViz 里 TF 短时间报警

如果只在刚启动时出现，通常是 Gazebo、map_server、AMCL 和 Nav2 生命周期还没全部激活。持续存在时检查地图是否加载成功、初始位姿是否在地图内，以及对应 namespace 下 AMCL 是否已经发布 `map -> odom`。

---

## 8. 项目表述示例

> 基于 ROS2 构建麦克纳姆轮机器人真机导航系统，完成激光接入、SLAM 建图、地图定位、Nav2 导航与键盘控制联调，形成从底盘运动验证到自主导航部署的完整流程。
