
# UAV A* Path Planning with ROS2 + PX4 + Gazebo

本项目用于实现基于 A* 算法的无人机路径规划，并结合 ROS2、PX4、Gazebo 和 RViz 完成路径规划、仿真飞行和轨迹显示。

## 项目结构

```text
uav-astar-px4-demo/
├── astar_planner/
│   ├── astar_node.py
│   └── astar_replay.py
├── gazebo_world/
│   └── astar_world.sdf
├── px4_control/
│   └── astar_px4_follower.py
└── scripts/
    └── restart_astar_flight.sh
```

## 文件说明

### astar_node.py

A* 路径规划节点，主要负责：

- 设置起点和终点
- 设置静态障碍物
- 进行 A* 路径搜索
- 发布规划路径
- 在 RViz 中显示障碍物、搜索节点和规划结果

### astar_replay.py

用于对 A* 路径进行动画回放，方便观察规划路径和运动过程。

### astar_px4_follower.py

用于接收 A* 规划路径，并通过 PX4 Offboard 模式控制 X500 无人机按照规划路径飞行。

同时记录无人机实际飞行位置，并将实际飞行轨迹发布到 RViz。

### astar_world.sdf

Gazebo 仿真环境文件。

当前环境中设置了多种静态障碍物，包括：

- 长方体
- 圆柱
- 长墙
- L 型障碍物
- U 型障碍物
- 方形障碍物

### restart_astar_flight.sh

用于重新启动 PX4 和 Gazebo 仿真环境，并重新执行飞行实验。

## 实验流程

### 1. 启动 MicroXRCEAgent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 2. 启动 PX4 和 Gazebo

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=astar_world make px4_sitl gz_x500
```

### 3. 启动 A* 路径规划节点

```bash
source /opt/ros/jazzy/setup.bash
source ~/astar_ws/install/setup.bash

ros2 run astar_planner astar_node
```

### 4. 启动 RViz

```bash
source /opt/ros/jazzy/setup.bash
source ~/astar_ws/install/setup.bash

rviz2
```

RViz 中设置：

```text
Fixed Frame = map
```

添加：

```text
/astar/markers
/astar/path
```

### 5. 启动 PX4 路径跟随节点

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_px4_ws/install/setup.bash

ros2 run uav_px4_control astar_px4_follower
```

启动后，X500 会按照 A* 规划结果依次经过各个航点，并最终到达终点。

## RViz 显示内容

```text
绿色路径：A* 规划路径
紫色路径：PX4 实际飞行轨迹
蓝色点：A* 搜索节点
绿色点：起点
红色点：终点
```

## 重新执行实验

重新运行 PX4 和 Gazebo：

```bash
~/restart_astar_flight.sh
```

## 当前实验结果

已完成以下流程：

```text
起点
↓
A* 路径规划
↓
避开静态障碍物
↓
生成规划路径
↓
PX4 Offboard 控制
↓
X500 按路径飞行
↓
RViz 显示实际飞行轨迹
↓
到达终点并悬停
```

## 开发环境

```text
Ubuntu 24.04
ROS 2 Jazzy
PX4 SITL
Gazebo Sim
QGroundControl
Python 3
```
