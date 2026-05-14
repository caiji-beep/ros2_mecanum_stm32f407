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

## 真机 EKF 融合

真机导航和真机建图支持可选 EKF。默认不开启，原始轮速 odom 仍可按旧流程使用：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  use_ekf:=true \
  can_interface:=can0
```

```bash
ros2 launch robmini_navigation demo_real_mapping.launch.py \
  use_ekf:=true \
  can_interface:=can0
```

启用后链路为：

```text
/robmini/odom
/robmini/imu/data_raw   # fuse_imu_yaw_rate:=true 时使用
  -> robot_localization/ekf_node
  -> /robmini/odometry/filtered
  -> odom -> base_link
```

为了避免 TF 冲突，`real_bringup.launch.py` 在 `use_ekf:=true` 时会自动关闭底盘控制器的 `enable_odom_tf`，由 EKF 发布 `odom -> base_link`。Nav2 会在运行时把 `odom_topic` 改为 `odometry/filtered`，原始 `config/nav2_params.yaml` 不会被改动。

当前 EKF 配置位于 `config/ekf_imu_odom.yaml`：

- 轮速 odom：融合 `x/y/yaw` 位姿和 `vx/vy/vyaw` 速度。
- CAN IMU：默认只在 `fuse_imu_yaw_rate:=true` 时融合 `angular_velocity.z`。
- 不融合下位机欧拉角 yaw，避免无磁力计约束时 yaw 漂移被当成绝对航向。

使用前需要安装：

```bash
sudo apt install ros-humble-robot-localization
```

只验证 EKF 与 Nav2 链路，不融合 IMU：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  use_ekf:=true \
  use_rviz:=false \
  fuse_imu_yaw_rate:=false
```

验证轮速 odom + IMU yaw rate 融合：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  use_ekf:=true \
  use_rviz:=false \
  fuse_imu_yaw_rate:=true
```

常用检查：

```bash
ros2 control list_controllers -c /robmini/controller_manager
ros2 topic hz /robmini/odom
ros2 topic hz /robmini/odometry/filtered
ros2 param get /robmini/ekf_filter_node odom0
ros2 param get /robmini/ekf_filter_node odom0_config
ros2 param get /robmini/ekf_filter_node imu0
ros2 param get /robmini/ekf_filter_node imu0_config
ros2 run tf2_ros tf2_echo robmini/odom robmini/base_link
```

判断是否真的用了 IMU：

- `fuse_imu_yaw_rate:=false` 时，`imu0` 相关参数会被 launch 移除，此时不是 IMU 融合，只是 EKF 接管 odom TF。
- `fuse_imu_yaw_rate:=true` 时，`imu0` 应为 `imu/data_raw`，`imu0_config` 中只有 yaw rate 对应项为 `true`。
- 若 `false` 和 `true` 导航效果接近，说明轮速 odom 本身已较稳定，IMU yaw rate 目前主要作为短时转向辅助。

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

### `use_ekf:=true` 后转弯乱或导航变差

如果 `use_ekf:=false` 时轮速 odom 导航正常，而 `use_ekf:=true` 后失败，即使 `fuse_imu_yaw_rate:=false` 仍失败，通常说明问题不在 IMU，而在 EKF 输出没有等价替代原始 `/robmini/odom`。

曾出现的问题是 EKF 只融合轮速 odom 的速度 `vx/vy/vyaw`，没有融合原始 odom 已经计算好的 `x/y/yaw` 位姿。这样 `fuse_imu_yaw_rate:=false` 并不等于回到原来的轮速 odom，而是 EKF 自己重新积分速度；一旦 EKF 频率不足、时间戳抖动或系统负载较高，`odom -> base_link` 就可能变差，Nav2 转弯时尤其明显。

当前临时稳定方案：

- `odom0_config` 同时融合轮速 odom 的 `x/y/yaw` 和 `vx/vy/vyaw`。
- EKF 频率设置为 `10Hz`，降低车载端负载。
- 先用 `fuse_imu_yaw_rate:=false` 验证 EKF 作为轮速 odom 替代链路稳定，再打开 `fuse_imu_yaw_rate:=true` 验证 IMU yaw rate 是否有收益。

推荐排查顺序：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  use_ekf:=true \
  use_rviz:=false \
  fuse_imu_yaw_rate:=false
```

确认能导航后再测试：

```bash
ros2 launch robmini_navigation demo_real_navigation.launch.py \
  use_ekf:=true \
  use_rviz:=false \
  fuse_imu_yaw_rate:=true
```

如果 `true` 明显比 `false` 差，重点检查 IMU `angular_velocity.z` 的单位、方向和零偏；如果二者差不多，可以保留当前融合结构。

### 多机共同建图后导航一开始规划失败

检查机器人初始位置是否落在融合地图的障碍或未知区域。共同建图保存地图时应保存 `/merged_map`，不要保存 `/robmini_01/map` 或 `/robmini_02/map`。

### 多机导航 RViz 刚启动时 TF 短暂报警

如果只在启动阶段出现，通常是 Gazebo、map_server、AMCL 和 Nav2 生命周期尚未全部激活。持续存在时检查地图是否加载成功、初始位姿是否在地图内，以及对应 namespace 下 AMCL 是否发布 `map -> odom`。
