# robmini_navigation

导航与交互核心包，负责激光接入、SLAM 建图、地图管理、Nav2 导航、RViz 配置、键盘控制，以及仿真多机器人导航/共同建图 demo。

## 目录

```text
robmini_navigation/
├── config/        # Nav2、SLAM、teleop 参数
├── launch/        # 单机器人/多机器人、真机/仿真启动文件
├── maps/          # 地图 yaml/pgm
├── rviz/          # RViz 配置
├── scripts/       # Python 辅助节点
└── src/           # C++ 节点
```

## 启动入口

| 场景 | 命令 |
| --- | --- |
| 真机建图 | `ros2 launch robmini_navigation demo_real_mapping.launch.py` |
| 真机导航 | `ros2 launch robmini_navigation demo_real_navigation.launch.py` |
| 仿真单机器人建图 | `ros2 launch robmini_navigation demo_sim_mapping.launch.py` |
| 仿真单机器人导航 | `ros2 launch robmini_navigation demo_sim_navigation.launch.py` |
| 仿真多机器人共同建图 | `ros2 launch robmini_navigation demo_sim_multi_robot_mapping.launch.py` |
| 仿真多机器人导航 | `ros2 launch robmini_navigation demo_sim_multi_robot_navigation.launch.py` |
| 键盘控制 | `ros2 launch robmini_navigation teleop.launch.py namespace:=robmini` |

地图路径 `map_file:=...` 相对 `robmini_navigation/maps/`，例如：

```bash
ros2 launch robmini_navigation demo_sim_navigation.launch.py \
  map_file:=room_mini/room_mini.yaml
```

## 键盘控制

默认键位：

| 按键 | 动作 |
| --- | --- |
| `w` / `s` | 前进 / 后退 |
| `a` / `d` | 左移 / 右移 |
| `q` / `e` | 左转 / 右转 |
| `x` | 停止 |

单机器人：

```bash
ros2 launch robmini_navigation teleop.launch.py \
  robot_name:=robmini \
  namespace:=robmini
```

多机器人分别开终端控制：

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

## 地图保存

单机器人建图保存：

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /robmini/map \
  -f /home/lsz/robotmini_ws/src/robmini_navigation/maps/room_mini/my_slam_map \
  --fmt pgm
```

多机器人共同建图保存：

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /merged_map \
  -f /home/lsz/robotmini_ws/src/robmini_navigation/maps/merged_multi/merged_map_01 \
  --fmt pgm
```

保存新地图后重新执行：

```bash
colcon build --symlink-install --packages-select robmini_navigation
source install/setup.bash
```

## 多机器人导航

`demo_sim_multi_robot_navigation.launch.py` 默认启动 `robmini_01` 和 `robmini_02` 两台车，每台车有独立 Nav2 namespace。

RViz 工具栏使用 `2D Goal Pose` 下发目标：

```text
/robmini_01/goal_pose
/robmini_02/goal_pose
```

`Publish Point` 的 `clicked_point` 只发布点坐标，不能触发 Nav2 导航。

加载融合地图：

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_navigation.launch.py \
  map_file:=merged_multi/merged_map_01.yaml
```

局部测试地图可以加载，但机器人初始位姿和目标点必须落在已建图区域。如果刚启动时缺少 `map -> odom`，通常是 AMCL 尚未定位成功；先确认对应 `/robmini_XX/map` 有数据，再用 RViz 的 `2D Pose Estimate` 设置初始位姿。

## 多机器人共同建图

`demo_sim_multi_robot_mapping.launch.py` 会启动两套独立 `slam_toolbox`：

```text
/robmini_01/map
/robmini_02/map
```

`scripts/simple_map_merge.py` 将两张局部地图合成为：

```text
/merged_map
```

当前方案直接使用原始 `/scan` 进入 `slam_toolbox`。同伴机器人造成的动态车身残影只在 `/merged_map` 输出前清理，不反向修改单车 SLAM 地图。

可调参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `clear_start_regions` | `true` | 清理两车启动位置附近残影 |
| `clear_start_radius` | `0.45` | 启动区域清理半径 |
| `clear_robot_trails` | `true` | 按机器人轨迹清理融合图 |
| `clear_robot_trail_radius` | `0.35` | 轨迹清理半径 |
| `map_origin_x_*/map_origin_y_*/map_origin_yaw_*` | `0` | 调整局部 map 到 `world` 的对齐关系 |

示例：

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_mapping.launch.py \
  clear_robot_trail_radius:=0.45 \
  clear_start_radius:=0.55
```

## 主要配置

| 文件 | 作用 |
| --- | --- |
| `config/nav2_params.yaml` | Nav2 主参数 |
| `config/slam_template.yaml` | SLAM 参数模板，支持按机器人名替换 |
| `config/teleop_params.yaml` | 键盘控制参数 |
| `rviz/nav2.rviz` | 单机器人导航 RViz |
| `rviz/multi_robots_navigation.rviz` | 多机器人导航 RViz |
| `rviz/multi_robots_mapping.rviz` | 多机器人建图 RViz |

## 常见问题

### 能键盘控制，但导航失败

优先检查雷达、TF、地图、Nav2 参数和 AMCL 初始位姿。底盘能被 teleop 控制，只说明控制链路基本正常。

### 多机共同建图后导航一开始规划失败

检查机器人初始位置是否落在融合地图的障碍或未知区域。共同建图保存地图时应保存 `/merged_map`，不要保存 `/robmini_01/map` 或 `/robmini_02/map`。

### 多机导航 RViz 刚启动时 TF 短暂报警

如果只在启动阶段出现，通常是 Gazebo、map_server、AMCL 和 Nav2 生命周期尚未全部激活。持续存在时检查地图是否加载成功、初始位姿是否在地图内，以及对应 namespace 下 AMCL 是否发布 `map -> odom`。
