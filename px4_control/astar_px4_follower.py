import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition
)

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)


class AStarPX4Follower(Node):

    def __init__(self):
        super().__init__('astar_px4_follower')

        # =====================================================
        # 飞行参数
        # =====================================================

        self.flight_height = 2.5
        self.reach_distance = 0.45

        # A*路径比较密，每4个点取一个
        self.sample_step = 4

        # =====================================================
        # QoS
        # =====================================================

        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # =====================================================
        # PX4发布器
        # =====================================================

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            self.px4_qos
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.px4_qos
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            self.px4_qos
        )

        # =====================================================
        # PX4位置
        # =====================================================

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            self.px4_qos
        )

        # =====================================================
        # A*路径
        # =====================================================

        self.path_sub = self.create_subscription(
            Path,
            '/astar/path',
            self.path_callback,
            self.path_qos
        )

        # =====================================================
        # RViz显示真实轨迹
        # =====================================================

        self.actual_path_pub = self.create_publisher(
            Marker,
            '/astar/px4_actual_path',
            10
        )

        # =====================================================
        # PX4当前位置
        #
        # PX4 NED：
        # x = North
        # y = East
        # z = Down
        # =====================================================

        self.current_north = 0.0
        self.current_east = 0.0
        self.current_down = 0.0

        self.have_position = False

        # =====================================================
        # 路径
        # =====================================================

        self.have_path = False
        self.waypoints = []

        self.current_waypoint = 0

        # =====================================================
        # 状态
        # =====================================================

        self.offboard_counter = 0

        self.started = False
        self.finished = False

        # =====================================================
        # 实际飞行轨迹
        # =====================================================

        self.actual_trail = []

        # =====================================================
        # 20Hz控制
        # =====================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            '======================================='
        )

        self.get_logger().info(
            'A* -> PX4 稳定版路径跟随节点启动'
        )

        self.get_logger().info(
            '等待A*路径和PX4位置...'
        )

        self.get_logger().info(
            '======================================='
        )

    # =========================================================
    # 接收A*路径
    # =========================================================

    def path_callback(self, msg):

        # 关键：A*会重复发布路径
        # 只允许第一次加载
        if self.have_path:
            return

        if len(msg.poses) == 0:
            return

        raw_path = []

        for pose in msg.poses:

            # RViz / Gazebo ENU
            #
            # x = East
            # y = North

            east = float(
                pose.pose.position.x
            )

            north = float(
                pose.pose.position.y
            )

            raw_path.append(
                (east, north)
            )

        # 降采样
        self.waypoints = raw_path[
            ::self.sample_step
        ]

        # 一定保留终点
        if self.waypoints[-1] != raw_path[-1]:

            self.waypoints.append(
                raw_path[-1]
            )

        self.current_waypoint = 0

        self.have_path = True

        self.get_logger().info(
            '======================================='
        )

        self.get_logger().info(
            f'A*原始路径：{len(raw_path)} 点'
        )

        self.get_logger().info(
            f'PX4使用：{len(self.waypoints)} 个航点'
        )

        self.get_logger().info(
            f'起点 East={self.waypoints[0][0]:.2f}, '
            f'North={self.waypoints[0][1]:.2f}'
        )

        self.get_logger().info(
            f'终点 East={self.waypoints[-1][0]:.2f}, '
            f'North={self.waypoints[-1][1]:.2f}'
        )

        self.get_logger().info(
            '======================================='
        )

    # =========================================================
    # PX4实际位置
    # =========================================================

    def position_callback(self, msg):

        self.current_north = float(msg.x)
        self.current_east = float(msg.y)
        self.current_down = float(msg.z)

        self.have_position = True

        # 正式飞行以后才记录轨迹
        if not self.started:
            return

        # =====================================================
        # PX4 NED -> RViz ENU
        #
        # RViz x = East
        # RViz y = North
        # =====================================================

        map_x = self.current_east
        map_y = self.current_north

        if len(self.actual_trail) == 0:

            self.actual_trail.append(
                (map_x, map_y)
            )

        else:

            last_x, last_y = (
                self.actual_trail[-1]
            )

            distance = math.hypot(
                map_x - last_x,
                map_y - last_y
            )

            if distance >= 0.05:

                self.actual_trail.append(
                    (map_x, map_y)
                )

        self.publish_actual_path()

    # =========================================================
    # 主循环
    # =========================================================

    def control_loop(self):

        # OFFBOARD模式必须持续发布
        self.publish_offboard_control_mode()

        if not self.have_path:
            return

        if not self.have_position:
            return

        # =====================================================
        # 已到终点
        # 持续发送最后航点，让无人机悬停
        # =====================================================

        if self.finished:

            final_east, final_north = (
                self.waypoints[-1]
            )

            self.publish_setpoint(
                final_east,
                final_north,
                self.flight_height
            )

            return

        # =====================================================
        # OFFBOARD之前先连续发送setpoint
        # =====================================================

        if self.offboard_counter < 20:

            first_east, first_north = (
                self.waypoints[0]
            )

            self.publish_setpoint(
                first_east,
                first_north,
                self.flight_height
            )

            self.offboard_counter += 1

            return

        # =====================================================
        # 第一次进入OFFBOARD
        # =====================================================

        if not self.started:

            self.set_offboard_mode()

            self.arm()

            self.started = True

            self.actual_trail = []

            self.get_logger().info(
                '======================================='
            )

            self.get_logger().info(
                '进入OFFBOARD'
            )

            self.get_logger().info(
                '开始执行A*路径'
            )

            self.get_logger().info(
                '======================================='
            )

            return

        # =====================================================
        # 防止越界
        # =====================================================

        if self.current_waypoint >= len(
            self.waypoints
        ):

            self.finished = True

            return

        # =====================================================
        # 当前航点
        # =====================================================

        target_east, target_north = (
            self.waypoints[
                self.current_waypoint
            ]
        )

        self.publish_setpoint(
            target_east,
            target_north,
            self.flight_height
        )

        # =====================================================
        # 距离计算
        # =====================================================

        east_error = (
            target_east
            -
            self.current_east
        )

        north_error = (
            target_north
            -
            self.current_north
        )

        distance = math.hypot(
            east_error,
            north_error
        )

        # =====================================================
        # 到达航点
        # =====================================================

        if distance <= self.reach_distance:

            self.get_logger().info(
                f'到达航点 '
                f'{self.current_waypoint + 1}'
                f'/'
                f'{len(self.waypoints)}'
            )

            self.current_waypoint += 1

            # =================================================
            # 到终点
            # =================================================

            if self.current_waypoint >= len(
                self.waypoints
            ):

                self.finished = True

                self.get_logger().info(
                    '======================================='
                )

                self.get_logger().info(
                    'A*路径飞行完成'
                )

                self.get_logger().info(
                    '已到达终点'
                )

                self.get_logger().info(
                    '终点悬停'
                )

                self.get_logger().info(
                    '======================================='
                )

    # =========================================================
    # ENU -> PX4 NED
    # =========================================================

    def publish_setpoint(
        self,
        east,
        north,
        height
    ):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        # =====================================================
        # RViz/Gazebo：
        #
        # x = East
        # y = North
        # z = Up
        #
        # PX4：
        #
        # x = North
        # y = East
        # z = Down
        # =====================================================

        msg.position = [
            float(north),
            float(east),
            float(-height)
        ]

        msg.yaw = 0.0

        self.trajectory_pub.publish(
            msg
        )

    # =========================================================
    # RViz实际轨迹
    # =========================================================

    def publish_actual_path(self):

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        marker.ns = 'px4_actual_path'
        marker.id = 0

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.15

        # 紫色
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        for x, y in self.actual_trail:

            point = Point()

            point.x = float(x)
            point.y = float(y)
            point.z = 0.30

            marker.points.append(
                point
            )

        self.actual_path_pub.publish(
            marker
        )

    # =========================================================
    # OFFBOARD模式
    # =========================================================

    def publish_offboard_control_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_pub.publish(
            msg
        )

    # =========================================================
    # 切换OFFBOARD
    # =========================================================

    def set_offboard_mode(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )

    # =========================================================
    # 解锁
    # =========================================================

    def arm(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )

    # =========================================================
    # PX4命令
    # =========================================================

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.command = command

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(
            msg
        )

    # =========================================================
    # 时间戳
    # =========================================================

    def timestamp(self):

        return int(
            self.get_clock()
            .now()
            .nanoseconds
            /
            1000
        )


def main(args=None):

    rclpy.init(args=args)

    node = AStarPX4Follower()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()