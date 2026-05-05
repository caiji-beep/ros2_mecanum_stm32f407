# 注意事项
仿真模式由于物理属性问题，左平移/右平移很难做到，故nav2的yaml关掉了y方向的速度。
然后引入了planer,当前仿真兼容planer和ros2_control，注意在使用ros2_control时，必须关掉nav2的yaml的y。
仿真时robmini.urdf.xacro  <xacro:property name="mesh_dir" value="file://$(find robmini_description)/meshes" />
实车且rviz2远程启动(本地启动可与仿真一致)时<xacro:property name="mesh_dir" value="package://robmini_description/meshes" />

# 快速检查

## 真实机器人启动

默认单机器人导航：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py
```

显式指定默认机器人：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  namespace:=robmini tf_prefix:=robmini
```

实车建图：

```bash
ros2 launch robmini_navigation demo_real_mapping.launch.py
```

```bash
ros2 launch robmini_navigation teleop.launch.py 
```

```bash
ros2 run nav2_map_server map_saver_cli -f \
/home/lsz/robotmini_ws/src/robmini_navigation/maps/room_mini/my_slam_map \
--ros-args -r map:=/robmini/map
```


## Gazebo 仿真启动

仿真入口和真机入口不同。仿真不会使用 `/dev/ttySTM32`，而是通过
`gazebo_ros2_control/GazeboSystem` 让 Gazebo 接管四个轮子的 velocity interface：
仿真导航
```bash
ros2 launch robmini_navigation demo_sim_navigation.launch.py
```

仿真建图过程（默认使用planer）:

```bash
ros2 launch robmini_navigation demo_real_mapping.launch.py
```

```bash
ros2 launch robmini_navigation teleop.launch.py 
```

```bash
ros2 run nav2_map_server map_saver_cli -f \
/home/lsz/robotmini_ws/src/robmini_navigation/maps/room_mini/my_slam_map \
--ros-args -r map:=/robmini/map
```



仿真链路大致是：

```text
Gazebo
  -> gazebo_ros2_control/GazeboSystem
  -> mecanum_drive_controller
  -> /robmini/odom
  -> Nav2
```

如果提示找不到 `libgazebo_ros2_control.so` 或 `gazebo_ros2_control/GazeboSystem`，
需要先安装：

```bash
sudo apt install ros-humble-gazebo-ros2-control
```

## Controller 状态检查

```bash
ros2 control list_controllers -c /robmini/controller_manager
ros2 control list_hardware_interfaces -c /robmini/controller_manager
```

正常情况下应该能看到：

```text
joint_state_broadcaster
mecanum_drive_controller
```

## 直接发送速度

controller active 后，可以直接发速度测试底盘：

```bash
ros2 topic pub /robmini/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}" --once
```

## RViz 远程显示

树莓派没有可视化界面时，推荐在虚拟机/远程 Ubuntu 上启动 RViz。
树莓派和虚拟机需要保持相同 DDS 配置，可以写入两边的 `~/.bashrc`：

```bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

默认 `/robmini` 单机器人可以继续用手动方式打开：

```bash
ros2 run rviz2 rviz2 \
  -d /home/lsz/robotmini_ws/install/robmini_navigation/share/robmini_navigation/rviz/nav2.rviz \
  --ros-args -r __ns:=/robmini -p use_sim_time:=false
```

也可以使用只启动 RViz 的 launch，它不会启动 Nav2、map_server 或机器人底盘节点：

```bash
ros2 launch robmini_navigation rviz.launch.py \
  namespace:=robmini tf_prefix:=robmini
```

如果不是默认机器人名，推荐使用这个 RViz launch 自动生成临时配置：

```bash
ros2 launch robmini_navigation rviz.launch.py \
  namespace:=robot1 tf_prefix:=robot1 map_frame:=map
```

## 多机器人示例

两台机器人共享同一张全局地图：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  robot_name:=robot1 namespace:=robot1 tf_prefix:=robot1 map_frame:=map

ros2 launch robmini_navigation demo_real_navigation.launch.py \
  robot_name:=robot2 namespace:=robot2 tf_prefix:=robot2 map_frame:=map
```

## CAN 与 IMU 检查

先启动 CAN：

```bash
sudo ip link set can0 up type can bitrate 125000
```

监听原始 CAN：

```bash
ros2 run can_socket_demo can_listener
```

启动 IMU 节点：

```bash
ros2 launch can_socket_demo can_imu.launch.py \
  namespace:=robmini tf_prefix:=robmini
```

IMU 默认发布到：

```text
/robmini/imu/data_raw
```

### 开机启动脚本
sudo nano /usr/local/bin/can0_up.sh

写入：

#!/bin/bash

sleep 5

modprobe can
modprobe can_raw
modprobe can_dev
modprobe peak_usb 2>/dev/null || true

for i in $(seq 1 15); do
    if ip link show can0 >/dev/null 2>&1; then
        ip link set can0 down 2>/dev/null || true
        ip link set can0 type can bitrate 125000 restart-ms 100
        ip link set can0 up
        ip -details -statistics link show can0
        exit 0
    fi
    sleep 1
done

echo "can0 not found"
exit 1

授权：

sudo chmod +x /usr/local/bin/can0_up.sh

创建服务：

sudo nano /etc/systemd/system/can0.service

写入：

[Unit]
Description=Bring up CAN0 interface
After=systemd-udev-settle.service
Wants=systemd-udev-settle.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/can0_up.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target

启用：

sudo systemctl daemon-reload
sudo systemctl enable can0.service
sudo systemctl start can0.service

以后树莓派开机后，can0 会自动 up。

