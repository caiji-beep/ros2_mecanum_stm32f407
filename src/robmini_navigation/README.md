# robmini_navigation（humble_real 分支）

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
│   └── room_mini/
├── rviz/
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

---

## 8. 项目表述示例

> 基于 ROS2 构建麦克纳姆轮机器人真机导航系统，完成激光接入、SLAM 建图、地图定位、Nav2 导航与键盘控制联调，形成从底盘运动验证到自主导航部署的完整流程。