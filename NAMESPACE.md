# 命名空间与 TF 约定

本项目把机器人标识拆成三个独立概念，避免多机器人时 topic、controller 和 TF 混在一起。

| 参数 | 作用 | 示例 |
| --- | --- | --- |
| `robot_name` | Gazebo entity 名称或默认机器人名 | `robmini_01` |
| `namespace` | ROS 图命名空间，隔离 topic、service、action、node | `/robmini_01` |
| `tf_prefix` | TF frame 前缀，隔离 `base_link`、`odom`、`laser` 等 frame | `robmini_01/base_link` |
| `map_frame` | Nav2、AMCL、SLAM 使用的全局地图 frame | `map` 或 `robmini_01/map` |

## 默认单机器人

默认启动：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py
```

等价于：

```text
robot_name = robmini
namespace  = robmini
tf_prefix  = robmini
map_frame  = robmini/map
```

也可以显式写出：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  robot_name:=robmini \
  namespace:=robmini \
  tf_prefix:=robmini
```

对应主要接口：

```text
/robmini/cmd_vel
/robmini/odom
/robmini/scan
/robmini/map
robmini/map -> robmini/odom -> robmini/base_link -> robmini/laser
```

## 多机器人共享地图

多机器人共用同一张全局地图时，推荐所有机器人使用同一个 `map` frame，同时用各自的 `namespace` 和 `tf_prefix` 隔离机器人本体。

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  robot_name:=robot1 \
  namespace:=robot1 \
  tf_prefix:=robot1 \
  map_frame:=map

ros2 launch robmini_navigation demo_real_navigation.launch.py \
  robot_name:=robot2 \
  namespace:=robot2 \
  tf_prefix:=robot2 \
  map_frame:=map
```

对应 TF 结构：

```text
map -> robot1/odom -> robot1/base_link -> robot1/laser
map -> robot2/odom -> robot2/base_link -> robot2/laser
```

## 多机器人独立地图

如果每台机器人各自建图或各自定位，不传 `map_frame:=map` 即可使用默认规则：

```text
map_frame = <tf_prefix>/map
```

例如：

```text
robmini_01/map -> robmini_01/odom -> robmini_01/base_link
robmini_02/map -> robmini_02/odom -> robmini_02/base_link
```

仿真多机器人共同建图中，两个局部地图会再通过静态关系对齐到统一的 `world` frame，并由 `simple_map_merge.py` 发布 `/merged_map`。

## 需要隔离的 ROS 接口

每台机器人自己的接口都应位于 namespace 下：

```text
/<namespace>/cmd_vel
/<namespace>/cmd_vel_nav
/<namespace>/odom
/<namespace>/scan
/<namespace>/imu/data_raw
/<namespace>/robot_description
/<namespace>/controller_manager
/<namespace>/mecanum_drive_controller/*
/<namespace>/map
/<namespace>/amcl
/<namespace>/planner_server
/<namespace>/controller_server
/<namespace>/bt_navigator
/<namespace>/local_costmap/*
/<namespace>/global_costmap/*
```

controller 名称本身不需要加机器人名前缀，因为它已经处在 namespace 下。多台机器人都可以叫：

```text
joint_state_broadcaster
mecanum_drive_controller
```

实际完整路径分别是：

```text
/robot1/joint_state_broadcaster
/robot1/mecanum_drive_controller
/robot2/joint_state_broadcaster
/robot2/mecanum_drive_controller
```

## TF 话题规则

TF 话题保持全局：

```text
/tf
/tf_static
```

不要改成 `/robot1/tf`、`/robot2/tf`。多机器人 TF 隔离靠 frame id 完成，例如 `robot1/base_link` 和 `robot2/base_link`。这样 Nav2、AMCL、SLAM 和 RViz 都能在同一棵 TF 树里看到完整关系。

## RViz 远程显示

树莓派或车载主机运行机器人栈，远程电脑运行 RViz 时，两边需要保持相同 DDS 配置：

```bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`ROS_LOCALHOST_ONLY` 必须为 `0`，否则 RViz 只能看到本机 ROS 节点。

默认 `/robmini` 可以直接打开包内 RViz 配置：

```bash
ros2 run rviz2 rviz2 \
  -d /home/lsz/robotmini_ws/install/robmini_navigation/share/robmini_navigation/rviz/nav2.rviz \
  --ros-args -r __ns:=/robmini -p use_sim_time:=false
```

非默认 namespace 时，优先使用对应场景的 demo launch 自带 RViz 配置；手动打开 RViz 时，需要同步修改 Fixed Frame、LaserScan、RobotModel、Nav2 Goal 等显示项和工具 Topic。
