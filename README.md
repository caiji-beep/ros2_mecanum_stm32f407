ros2 topic pub /robmini/mecanum_drive_controller_test01/reference \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'robmini/base_link'},
    twist: {linear: {x: -0.13, y: 0.0, z: 0.0},
            angular:{x: 0.0,  y: 0.0, z: 0.0}}}" --once


ros2 topic pub --rate 10 /robmini/mecanum_cont/cmd_vel geometry_msgs/msg/TwistStamped "
    twist:
      linear:
        x: 5.7
        y: 2.0
        z: 0.0
      angular:
        x: 0.0
        y: 0.0
        z: 1.0"
ros2 topic pub /robmini/mecanum_drive_controller_test01/reference_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once


ros2 control list_controllers -c /robmini/controller_manager
ros2 control list_hardware_interfaces -c /robmini/controller_manager
# 看到四个 wheel_joint/velocity 是 [claimed] 就对了


# 方式A：直接运行（脚本首行有 #!/usr/bin/env python3）
~/robotmini_ws/scripts/stm32_dummy_feeder.py --dev /tmp/ttyV1 --baud 115200 --mode echo --rate 50

# 方式B：用 python3 运行（即使没执行权限也能用这种方式）


python3 ~/robotmini_ws/scripts/stm32_dummy_feeder.py \
  --port /tmp/ttyV1 --baud 115200 --rate 5

  stdbuf -o0 -e0 socat -x -v -d -d \
  pty,link=/tmp/ttyV0,raw,echo=0,mode=666 \
  pty,link=/tmp/ttyV1,raw,echo=0,mode=666 \
  2>&1 | tee /tmp/socat.all.log

python3 ~/robotmini_ws/scripts/stm32_dummy_feeder.py --port /tmp/ttyV1 --baud 115200 --rate 30

ros2 launch robmini_description real_bringup.launch.py

ros2 launch robmini_navigation demo_real_navigation.launch.py 

ros2 run rviz2 rviz2 -d /home/lsz/robotmini_ws/install/robmini_navigation/share/robmini_navigation/rviz/nav2.rviz --ros-args -r __ns:=/robmini -p use_sim_time:=false

sudo poweroff

can_demo:
1.sudo ip link set can0 up type can bitrate 125000
2.ros2 run can_socket_demo can_listener