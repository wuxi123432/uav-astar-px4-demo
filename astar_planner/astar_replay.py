import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)


class AStarReplay(Node):

    def __init__(self):
        super().__init__('astar_replay')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 接收A*路径
        self.path_sub = self.create_subscription(
            Path,
            '/astar/path',
            self.path_callback,
            qos
        )

        # 发布运动小球和运动轨迹
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/astar/replay_markers',
            10
        )

        # 重新播放服务
        self.replay_service = self.create_service(
            Trigger,
            '/astar/replay',
            self.replay_callback
        )

        self.path = []

        self.current_index = 0

        self.running = False

        self.trail = []

        # 每0.08秒运动一次
        self.timer = self.create_timer(
            0.08,
            self.timer_callback
        )

        self.get_logger().info(
            'A*轨迹回放节点启动成功'
        )

    # ===============================
    # 接收路径
    # ===============================

    def path_callback(self, msg):

        if len(msg.poses) == 0:
            return

        self.path = []

        for pose in msg.poses:

            x = pose.pose.position.x
            y = pose.pose.position.y

            self.path.append(
                (x, y)
            )

        self.get_logger().info(
            f'收到A*路径，共 {len(self.path)} 个路径点'
        )

        # 第一次收到路径自动播放
        if not self.running:

            self.start_replay()

    # ===============================
    # 开始重新播放
    # ===============================

    def start_replay(self):

        if len(self.path) == 0:

            self.get_logger().warn(
                '目前还没有收到A*路径'
            )

            return False

        self.current_index = 0

        self.trail = []

        self.running = True

        self.get_logger().info(
            '开始播放A*运动轨迹'
        )

        return True

    # ===============================
    # ROS2服务
    # ===============================

    def replay_callback(
        self,
        request,
        response
    ):

        if self.start_replay():

            response.success = True

            response.message = (
                'A*轨迹重新播放'
            )

        else:

            response.success = False

            response.message = (
                '没有可播放的路径'
            )

        return response

    # ===============================
    # 定时运动
    # ===============================

    def timer_callback(self):

        if not self.running:
            return

        if self.current_index >= len(self.path):

            self.running = False

            self.get_logger().info(
                '无人机已到达终点'
            )

            return

        x, y = self.path[
            self.current_index
        ]

        self.trail.append(
            (x, y)
        )

        self.publish_markers(
            x,
            y
        )

        self.current_index += 1

    # ===============================
    # 发布运动小球和轨迹
    # ===============================

    def publish_markers(
        self,
        x,
        y
    ):

        marker_array = MarkerArray()

        now = (
            self.get_clock()
            .now()
            .to_msg()
        )

        # ===========================
        # 1. 无人机小球
        # ===========================

        drone = Marker()

        drone.header.frame_id = 'map'
        drone.header.stamp = now

        drone.ns = 'drone'
        drone.id = 0

        drone.type = Marker.SPHERE
        drone.action = Marker.ADD

        drone.pose.position.x = x
        drone.pose.position.y = y
        drone.pose.position.z = 0.35

        drone.pose.orientation.w = 1.0

        drone.scale.x = 0.45
        drone.scale.y = 0.45
        drone.scale.z = 0.45

        drone.color.r = 1.0
        drone.color.g = 1.0
        drone.color.b = 0.0
        drone.color.a = 1.0

        marker_array.markers.append(
            drone
        )

        # ===========================
        # 2. 已经走过的轨迹
        # ===========================

        trail_marker = Marker()

        trail_marker.header.frame_id = 'map'
        trail_marker.header.stamp = now

        trail_marker.ns = 'trail'
        trail_marker.id = 1

        trail_marker.type = Marker.LINE_STRIP
        trail_marker.action = Marker.ADD

        trail_marker.scale.x = 0.18

        trail_marker.color.r = 1.0
        trail_marker.color.g = 0.0
        trail_marker.color.b = 1.0
        trail_marker.color.a = 1.0

        for tx, ty in self.trail:

            point = Point()

            point.x = tx
            point.y = ty
            point.z = 0.25

            trail_marker.points.append(
                point
            )

        marker_array.markers.append(
            trail_marker
        )

        self.marker_pub.publish(
            marker_array
        )


def main(args=None):

    rclpy.init(args=args)

    node = AStarReplay()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
