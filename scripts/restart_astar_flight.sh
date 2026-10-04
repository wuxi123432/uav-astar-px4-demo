#!/bin/bash

echo "======================================"
echo "A* + PX4 重新飞行"
echo "======================================"

# 1. 停止旧的路径跟随节点
echo "[1/5] 停止旧的 A* PX4 跟随节点..."
pkill -f "astar_px4_follower" 2>/dev/null || true

sleep 1

# 2. 停止旧 PX4 / Gazebo
echo "[2/5] 关闭旧 PX4 / Gazebo..."

pkill -f px4 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true

sleep 3

# 3. 启动新的 PX4 + A* 障碍物世界
echo "[3/5] 重新启动 PX4 + astar_world..."

gnome-terminal --title="PX4 + AStar World" -- bash -c '
cd ~/PX4-Autopilot
PX4_GZ_WORLD=astar_world make px4_sitl gz_x500
exec bash
'

echo "等待 PX4 / Gazebo 启动..."
sleep 10

# 4. 等待 ROS2 收到 PX4 位置
echo "[4/5] 检查 PX4 -> ROS2 通信..."

source /opt/ros/jazzy/setup.bash
source ~/ros2_px4_ws/install/setup.bash

for i in {1..20}
do
    if timeout 2 ros2 topic echo \
        /fmu/out/vehicle_local_position_v1 \
        --once >/dev/null 2>&1
    then
        echo "PX4 位置数据已恢复！"
        break
    fi

    echo "等待 PX4 数据... $i/20"
    sleep 1
done

# 5. 自动启动 A* 跟随节点
echo "[5/5] 启动 A* PX4 路径跟随..."

gnome-terminal --title="AStar PX4 Follower" -- bash -c '
source /opt/ros/jazzy/setup.bash
source ~/ros2_px4_ws/install/setup.bash

ros2 run uav_px4_control astar_px4_follower

exec bash
'

echo ""
echo "======================================"
echo "重新飞行流程已启动"
echo "请观察 Gazebo 和 RViz"
echo "======================================"
