# 命名空间与 TF 约定

本项目现在把以前混在一起的 `robot_name` 拆成三个概念：

- `namespace`：ROS 图命名空间，用来隔离 topic、service、action、node 名称。
- `tf_prefix`：TF frame 前缀，用来生成 `robot1/base_link` 这类 frame id。
- `map_frame`：定位和导航使用的全局坐标系。

## 默认单机器人

默认启动命令保持不变：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py
```

上面这个命令等价于：

```text
namespace = robmini
tf_prefix = robmini
map_frame = robmini/map
```

也可以显式写出来：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  namespace:=robmini \
  tf_prefix:=robmini
```

## 多机器人共享地图

如果多个机器人共用同一张全局地图，推荐让所有机器人使用同一个 `map` frame，
每台机器人只隔离自己的 namespace 和 TF 前缀：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  robot_name:=robot1 namespace:=robot1 tf_prefix:=robot1 map_frame:=map

ros2 launch robmini_navigation demo_real_navigation.launch.py \
  robot_name:=robot2 namespace:=robot2 tf_prefix:=robot2 map_frame:=map
```

对应 TF 结构大致是：

```text
map -> robot1/odom -> robot1/base_link -> robot1/laser
map -> robot2/odom -> robot2/base_link -> robot2/laser
```

如果希望每台机器人各自拥有独立地图，不传 `map_frame:=map` 即可，默认会变成：

```text
<tf_prefix>/map
```

例如默认机器人就是 `robmini/map`。

## 每台机器人需要隔离的接口

每台机器人自己的 ROS 接口都应该在 namespace 下：

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

## Controller 是否需要命名空间

需要。

多机器人时，`controller_manager`、controller 的 service、controller 内部 topic 都必须隔离。
本项目采用下面这种形式：

```text
/<namespace>/controller_manager
/<namespace>/joint_state_broadcaster
/<namespace>/mecanum_drive_controller
```

controller 名称本身不需要加机器人名前缀，因为它已经处在各自 namespace 下。
也就是说，多台机器人都可以叫 `mecanum_drive_controller`，只要分别位于：

```text
/robot1/mecanum_drive_controller
/robot2/mecanum_drive_controller
```

旧的临时名称已经不再使用：

```text
joint_broad_test01
mecanum_drive_controller_test01
joint_broad_test02
mecanum_cont_test02
```

现在统一使用：

```text
joint_state_broadcaster
mecanum_drive_controller
```

## TF 话题怎么处理

TF 话题保持全局：

```text
/tf
/tf_static
```

不要把 TF 话题改成 `/robot1/tf`、`/robot2/tf`。隔离靠 frame id 完成，
例如 `robot1/base_link` 和 `robot2/base_link`。

这样 Nav2、RViz、AMCL、SLAM 都能在同一个 TF 树里看到完整关系。

## RViz 远程显示建议

树莓派没有可视化界面时，机器人栈运行在树莓派，RViz 可以运行在虚拟机/远程 Ubuntu。
两边需要保持相同 DDS 配置，例如写入 `~/.bashrc`：

```bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`ROS_LOCALHOST_ONLY` 必须是 `0`，否则 RViz 只能看到本机 ROS 节点，看不到树莓派上的 topic。

`rviz/` 目录下的配置文件仍然保留 `/robmini` 作为默认模板，方便默认单机器人直接打开。
默认 `/robmini` 可以使用你现在的手动方式：

```bash
ros2 run rviz2 rviz2 \
  -d /home/lsz/robotmini_ws/install/robmini_navigation/share/robmini_navigation/rviz/nav2.rviz \
  --ros-args -r __ns:=/robmini -p use_sim_time:=false
```

如果不是默认 namespace，推荐在虚拟机上使用只启动 RViz 的 launch，启动文件会自动生成临时 RViz 配置，
不会重复启动 Nav2 或底盘节点：

```bash
ros2 launch robmini_navigation rviz.launch.py \
  namespace:=robot1 tf_prefix:=robot1 map_frame:=map
```
