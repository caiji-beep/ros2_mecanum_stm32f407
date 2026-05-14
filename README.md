# robotmini_ws

ROS2 Humble 麦克纳姆轮机器人工作区，包含真机底盘接入、RPLIDAR 激光、SLAM 建图、Nav2 导航、Gazebo 仿真、多机器人导航和多机器人共同建图。

## 文档分工

| 文档 | 作用 |
| --- | --- |
| `README.md` | 项目总览、环境、编译和常用启动命令 |
| `NAMESPACE.md` | `namespace`、`tf_prefix`、`map_frame` 与多机器人 TF 约定 |
| `src/<package>/README.md` | 单个功能包的实现细节、接口和排障说明 |

## 目录结构

```text
robotmini_ws/
├── src/
│   ├── can_socket_demo/          # CAN 与 IMU 调试辅助
│   ├── mecanum_hw_interface/     # ros2_control 真机串口硬件接口
│   ├── robmini_description/      # URDF/Xacro、控制器配置、真机/仿真 bringup
│   ├── robmini_navigation/       # SLAM、Nav2、RViz、teleop、多机器人 demo
│   └── rplidar_ros/              # SLAMTEC RPLIDAR 驱动
├── NAMESPACE.md
└── README.md
```

## 环境

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic
- Nav2
- slam_toolbox
- ros2_control
- rviz2
- colcon

可按需安装常用依赖：

```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-robot-localization \
  ros-humble-xacro \
  ros-humble-rviz2
```

## 编译

在工作区根目录执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

每次新增地图、RViz 配置或 launch 文件后，建议重新 build 并 source。

## 常用启动

### 仿真单机器人导航

```bash
ros2 launch robmini_navigation demo_sim_navigation.launch.py
```

默认加载 `src/robmini_navigation/maps/room_mini/room_mini.yaml`。

指定地图：

```bash
ros2 launch robmini_navigation demo_sim_navigation.launch.py \
  map_file:=room_mini/room_mini.yaml
```

### 仿真单机器人建图

```bash
ros2 launch robmini_navigation demo_sim_mapping.launch.py
```

另开终端控制小车：

```bash
ros2 launch robmini_navigation teleop.launch.py \
  robot_name:=robmini \
  namespace:=robmini
```

保存单机器人地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /robmini/map \
  -f /home/lsz/robotmini_ws/src/robmini_navigation/maps/room_mini/my_slam_map \
  --fmt pgm
```

### 仿真多机器人导航

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_navigation.launch.py
```

RViz 工具栏使用 `2D Goal Pose` 下发目标。默认 Topic 为 `/robmini_01/goal_pose`；指挥 2 号车时，在 Tool Properties 中改为 `/robmini_02/goal_pose`。`Publish Point` 的 `clicked_point` 只发布点坐标，不能触发 Nav2 导航。

加载共同建图保存的融合地图：

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_navigation.launch.py \
  map_file:=merged_multi/merged_map_01.yaml
```

### 仿真多机器人共同建图

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_mapping.launch.py
```

分别控制两台车：

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

该流程会启动两套独立 `slam_toolbox`，并将 `/robmini_01/map`、`/robmini_02/map` 合成为 `/merged_map`。保存融合地图时请保存 `/merged_map`，不要保存单车 map：

```bash
mkdir -p /home/lsz/robotmini_ws/src/robmini_navigation/maps/merged_multi

ros2 run nav2_map_server map_saver_cli \
  -t /merged_map \
  -f /home/lsz/robotmini_ws/src/robmini_navigation/maps/merged_multi/merged_map_01 \
  --fmt pgm
```

如果融合图上仍有车身残影，可增大清理半径：

```bash
ros2 launch robmini_navigation demo_sim_multi_robot_mapping.launch.py \
  clear_robot_trail_radius:=0.45 \
  clear_start_radius:=0.55
```

### 真机建图

```bash
ros2 launch robmini_navigation demo_real_mapping.launch.py
```

另开终端使用键盘控制：

```bash
ros2 launch robmini_navigation teleop.launch.py \
  robot_name:=robmini \
  namespace:=robmini
```

保存真机地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /robmini/map \
  -f /home/lsz/robotmini_ws/src/robmini_navigation/maps/room_mini/my_real_map \
  --fmt pgm
```

### 真机导航

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py
```

指定地图：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  map_file:=room_mini/115_map.yaml
```

启用轮速 odom + CAN IMU 的 EKF 融合：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  use_ekf:=true \
  can_interface:=can0
```

只验证 EKF 链路、不融合 IMU 时：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  use_ekf:=true \
  use_rviz:=false \
  fuse_imu_yaw_rate:=false
```

启用后底盘控制器不再发布 `odom -> base_link`，由 `robot_localization` 发布融合后的 TF，并输出 `/robmini/odometry/filtered`。当前 EKF 融合轮速 odom 的 `x/y/yaw` 和 `vx/vy/vyaw`，开启 `fuse_imu_yaw_rate:=true` 时再额外融合 CAN IMU 的 `angular_velocity.z`。不使用无磁力计约束的下位机 yaw 欧拉角。

检查融合链路：

```bash
ros2 topic hz /robmini/odom
ros2 topic hz /robmini/odometry/filtered
ros2 param get /robmini/ekf_filter_node odom0_config
ros2 param get /robmini/ekf_filter_node imu0_config
ros2 run tf2_ros tf2_echo robmini/odom robmini/base_link
```

相关问题原因和调试流程见 `src/robmini_navigation/README.md` 的“真机 EKF 融合”和“常见问题”。

树莓派或车载主机无显示时，可只在远程电脑启动 RViz。远程显示和多机器人命名空间规则见 `NAMESPACE.md`。

## 关键约定

- 默认机器人名、命名空间和 TF 前缀为 `robmini`。
- 多机器人时用 `namespace` 隔离 ROS topic/service/action，用 `tf_prefix` 隔离 TF frame。
- TF 话题保持全局 `/tf`、`/tf_static`，不要改成 `/robot1/tf` 这类私有话题。
- 地图路径 `map_file:=...` 相对 `src/robmini_navigation/maps/`。
- 仿真默认使用 `sim_drive_mode:=planar`，更适合 Nav2 验证；需要 ros2_control 仿真链路时可切换为 `sim_drive_mode:=ros2_control`。
- 多机器人共同建图不再过滤送入 `slam_toolbox` 的 `/scan`，动态车身残影只在 `/merged_map` 输出前清理。

## 常用检查

检查 controller：

```bash
ros2 control list_controllers -c /robmini/controller_manager
ros2 control list_hardware_interfaces -c /robmini/controller_manager
```

直接发送速度：

```bash
ros2 topic pub /robmini/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}" --once
```

检查 CAN：

```bash
sudo ip link set can0 up type can bitrate 125000
ros2 run can_socket_demo can_listener
```

启动 CAN IMU：

```bash
ros2 launch can_socket_demo can_imu.launch.py \
  namespace:=robmini \
  tf_prefix:=robmini
```
