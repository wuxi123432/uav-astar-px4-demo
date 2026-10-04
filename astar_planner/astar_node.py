import math
import heapq

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


class AStarNode(Node):

    def __init__(self):
        super().__init__('astar_node')

        # =========================
        # 1. 地图参数
        # =========================
        self.resolution = 0.25

        self.map_width = 16.0
        self.map_height = 12.0

        # 起点和终点
        self.start_world = (0.5, 0.5)
        self.goal_world = (14.5, 10.5)

        # 障碍物安全距离
        self.safe_distance = 0.6

        # =========================
        # 2. ROS2 发布器
        # =========================
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.path_pub = self.create_publisher(
            Path,
            '/astar/path',
            qos
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/astar/markers',
            qos
        )

        # =========================
        # 3. 创建障碍物
        # =========================
        self.raw_obstacles = set()

        self.create_obstacles()

        # =========================
        # 4. 给障碍物增加安全距离
        # =========================
        self.inflated_obstacles = self.inflate_obstacles(
            self.raw_obstacles,
            self.safe_distance
        )

        # =========================
        # 5. A*规划
        # =========================
        self.visited_nodes = []

        self.path = self.astar(
            self.start_world,
            self.goal_world
        )

        if self.path:
            self.get_logger().info(
                f'A*规划成功！路径点数量：{len(self.path)}'
            )

            length = self.calculate_path_length(self.path)

            self.get_logger().info(
                f'路径长度：{length:.2f} m'
            )
        else:
            self.get_logger().error(
                'A*规划失败，没有找到路径！'
            )

        # 每秒重新发布一次，方便之后打开RViz还能看到
        self.timer = self.create_timer(
            1.0,
            self.publish_all
        )

        self.publish_all()

    # =====================================================
    # 坐标转换
    # =====================================================

    def world_to_grid(self, x, y):

        gx = int(round(x / self.resolution))
        gy = int(round(y / self.resolution))

        return gx, gy

    def grid_to_world(self, gx, gy):

        x = gx * self.resolution
        y = gy * self.resolution

        return x, y

    # =====================================================
    # 创建障碍物
    # =====================================================

    def create_obstacles(self):

        """
        一共6组障碍物：

        1 长方体
        2 圆柱
        3 长墙
        4 L形
        5 U形
        6 方形建筑
        """

        # -------------------------
        # 障碍物1：长方形
        # -------------------------
        self.add_rectangle(
            2.5, 1.5,
            3.8, 4.5
        )

        # -------------------------
        # 障碍物2：圆形
        # -------------------------
        self.add_circle(
            6.3,
            8.0,
            1.0
        )

        # -------------------------
        # 障碍物3：长墙
        # -------------------------
        self.add_rectangle(
            4.5, 5.3,
            8.0, 5.8
        )

        # -------------------------
        # 障碍物4：L形
        # -------------------------

        # L竖边
        self.add_rectangle(
            8.0, 2.0,
            8.7, 5.0
        )

        # L横边
        self.add_rectangle(
            8.0, 2.0,
            10.5, 2.7
        )

        # -------------------------
        # 障碍物5：U形
        # -------------------------

        # U左边
        self.add_rectangle(
            10.0, 4.5,
            10.7, 7.5
        )

        # U右边
        self.add_rectangle(
            12.5, 4.5,
            13.2, 7.5
        )

        # U底边
        self.add_rectangle(
            10.0, 4.5,
            13.2, 5.2
        )

        # -------------------------
        # 障碍物6：方形建筑
        # -------------------------
        self.add_rectangle(
            11.0, 8.3,
            12.7, 9.8
        )

    # =====================================================
    # 添加矩形障碍
    # =====================================================

    def add_rectangle(self, xmin, ymin, xmax, ymax):

        gx_min, gy_min = self.world_to_grid(
            xmin,
            ymin
        )

        gx_max, gy_max = self.world_to_grid(
            xmax,
            ymax
        )

        for gx in range(gx_min, gx_max + 1):

            for gy in range(gy_min, gy_max + 1):

                self.raw_obstacles.add(
                    (gx, gy)
                )

    # =====================================================
    # 添加圆形障碍
    # =====================================================

    def add_circle(self, cx, cy, radius):

        gx_min, gy_min = self.world_to_grid(
            cx - radius,
            cy - radius
        )

        gx_max, gy_max = self.world_to_grid(
            cx + radius,
            cy + radius
        )

        for gx in range(gx_min, gx_max + 1):

            for gy in range(gy_min, gy_max + 1):

                x, y = self.grid_to_world(
                    gx,
                    gy
                )

                distance = math.hypot(
                    x - cx,
                    y - cy
                )

                if distance <= radius:

                    self.raw_obstacles.add(
                        (gx, gy)
                    )

    # =====================================================
    # 障碍物膨胀
    # =====================================================

    def inflate_obstacles(
        self,
        obstacles,
        safe_distance
    ):

        inflated = set(obstacles)

        cells = int(
            math.ceil(
                safe_distance /
                self.resolution
            )
        )

        for ox, oy in obstacles:

            for dx in range(
                -cells,
                cells + 1
            ):

                for dy in range(
                    -cells,
                    cells + 1
                ):

                    distance = math.hypot(
                        dx * self.resolution,
                        dy * self.resolution
                    )

                    if distance <= safe_distance:

                        nx = ox + dx
                        ny = oy + dy

                        if self.valid_grid(nx, ny):

                            inflated.add(
                                (nx, ny)
                            )

        return inflated

    # =====================================================
    # 判断地图范围
    # =====================================================

    def valid_grid(self, gx, gy):

        max_x = int(
            self.map_width /
            self.resolution
        )

        max_y = int(
            self.map_height /
            self.resolution
        )

        return (
            0 <= gx <= max_x
            and
            0 <= gy <= max_y
        )

    # =====================================================
    # A*算法
    # =====================================================

    def astar(self, start_world, goal_world):

        start = self.world_to_grid(
            *start_world
        )

        goal = self.world_to_grid(
            *goal_world
        )

        if start in self.inflated_obstacles:

            self.get_logger().error(
                '起点在障碍物内！'
            )

            return []

        if goal in self.inflated_obstacles:

            self.get_logger().error(
                '终点在障碍物内！'
            )

            return []

        # 8方向移动
        motions = [

            (1, 0, 1.0),
            (-1, 0, 1.0),

            (0, 1, 1.0),
            (0, -1, 1.0),

            (1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),

            (-1, 1, math.sqrt(2)),
            (-1, -1, math.sqrt(2))
        ]

        open_heap = []

        heapq.heappush(
            open_heap,
            (
                0.0,
                start
            )
        )

        came_from = {}

        g_score = {
            start: 0.0
        }

        closed = set()

        self.visited_nodes = []

        while open_heap:

            _, current = heapq.heappop(
                open_heap
            )

            if current in closed:
                continue

            closed.add(current)

            self.visited_nodes.append(
                current
            )

            # 找到终点
            if current == goal:

                return self.reconstruct_path(
                    came_from,
                    current
                )

            for dx, dy, move_cost in motions:

                nx = current[0] + dx
                ny = current[1] + dy

                neighbour = (
                    nx,
                    ny
                )

                # 超出地图
                if not self.valid_grid(
                    nx,
                    ny
                ):
                    continue

                # 撞障碍物
                if neighbour in self.inflated_obstacles:
                    continue

                # -------------------------
                # 防止斜着穿过障碍物拐角
                # -------------------------
                if dx != 0 and dy != 0:

                    side1 = (
                        current[0] + dx,
                        current[1]
                    )

                    side2 = (
                        current[0],
                        current[1] + dy
                    )

                    if (
                        side1 in self.inflated_obstacles
                        or
                        side2 in self.inflated_obstacles
                    ):
                        continue

                tentative_g = (
                    g_score[current]
                    +
                    move_cost
                )

                if tentative_g < g_score.get(
                    neighbour,
                    float('inf')
                ):

                    came_from[
                        neighbour
                    ] = current

                    g_score[
                        neighbour
                    ] = tentative_g

                    h = self.heuristic(
                        neighbour,
                        goal
                    )

                    f = tentative_g + h

                    heapq.heappush(
                        open_heap,
                        (
                            f,
                            neighbour
                        )
                    )

        return []

    # =====================================================
    # A*启发函数
    # =====================================================

    def heuristic(self, a, b):

        dx = abs(
            a[0] - b[0]
        )

        dy = abs(
            a[1] - b[1]
        )

        # 八方向 Octile Distance
        return (
            dx
            +
            dy
            +
            (
                math.sqrt(2) - 2
            )
            *
            min(dx, dy)
        )

    # =====================================================
    # 重建路径
    # =====================================================

    def reconstruct_path(
        self,
        came_from,
        current
    ):

        grid_path = [
            current
        ]

        while current in came_from:

            current = came_from[
                current
            ]

            grid_path.append(
                current
            )

        grid_path.reverse()

        world_path = []

        for gx, gy in grid_path:

            x, y = self.grid_to_world(
                gx,
                gy
            )

            world_path.append(
                (x, y)
            )

        return world_path

    # =====================================================
    # 路径长度
    # =====================================================

    def calculate_path_length(
        self,
        path
    ):

        total = 0.0

        for i in range(
            len(path) - 1
        ):

            x1, y1 = path[i]
            x2, y2 = path[i + 1]

            total += math.hypot(
                x2 - x1,
                y2 - y1
            )

        return total

    # =====================================================
    # 发布全部信息
    # =====================================================

    def publish_all(self):

        self.publish_path()

        self.publish_markers()

    # =====================================================
    # 发布路径
    # =====================================================

    def publish_path(self):

        path_msg = Path()

        path_msg.header.frame_id = 'map'

        path_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        for x, y in self.path:

            pose = PoseStamped()

            pose.header = path_msg.header

            pose.pose.position.x = x
            pose.pose.position.y = y

            pose.pose.position.z = 0.15

            pose.pose.orientation.w = 1.0

            path_msg.poses.append(
                pose
            )

        self.path_pub.publish(
            path_msg
        )

    # =====================================================
    # 发布RViz标记
    # =====================================================

    def publish_markers(self):

        markers = MarkerArray()

        now = (
            self.get_clock()
            .now()
            .to_msg()
        )

        # =================================================
        # 1 原始障碍物
        # =================================================

        obstacle_marker = Marker()

        obstacle_marker.header.frame_id = 'map'
        obstacle_marker.header.stamp = now

        obstacle_marker.ns = 'obstacles'
        obstacle_marker.id = 0

        obstacle_marker.type = Marker.CUBE_LIST
        obstacle_marker.action = Marker.ADD

        obstacle_marker.scale.x = self.resolution
        obstacle_marker.scale.y = self.resolution
        obstacle_marker.scale.z = 1.5

        obstacle_marker.color.r = 0.25
        obstacle_marker.color.g = 0.25
        obstacle_marker.color.b = 0.25
        obstacle_marker.color.a = 1.0

        for gx, gy in self.raw_obstacles:

            x, y = self.grid_to_world(
                gx,
                gy
            )

            p = Point()

            p.x = x
            p.y = y
            p.z = 0.75

            obstacle_marker.points.append(
                p
            )

        markers.markers.append(
            obstacle_marker
        )

        # =================================================
        # 2 安全膨胀区
        # =================================================

        safe_marker = Marker()

        safe_marker.header.frame_id = 'map'
        safe_marker.header.stamp = now

        safe_marker.ns = 'safe_zone'
        safe_marker.id = 1

        safe_marker.type = Marker.CUBE_LIST
        safe_marker.action = Marker.ADD

        safe_marker.scale.x = self.resolution
        safe_marker.scale.y = self.resolution
        safe_marker.scale.z = 0.05

        safe_marker.color.r = 1.0
        safe_marker.color.g = 0.5
        safe_marker.color.b = 0.0
        safe_marker.color.a = 0.25

        safe_only = (
            self.inflated_obstacles
            -
            self.raw_obstacles
        )

        for gx, gy in safe_only:

            x, y = self.grid_to_world(
                gx,
                gy
            )

            p = Point()

            p.x = x
            p.y = y
            p.z = 0.03

            safe_marker.points.append(
                p
            )

        markers.markers.append(
            safe_marker
        )

        # =================================================
        # 3 A*搜索过的节点
        # =================================================

        visited_marker = Marker()

        visited_marker.header.frame_id = 'map'
        visited_marker.header.stamp = now

        visited_marker.ns = 'visited'
        visited_marker.id = 2

        visited_marker.type = Marker.POINTS
        visited_marker.action = Marker.ADD

        visited_marker.scale.x = 0.07
        visited_marker.scale.y = 0.07

        visited_marker.color.r = 0.3
        visited_marker.color.g = 0.6
        visited_marker.color.b = 1.0
        visited_marker.color.a = 0.65

        for gx, gy in self.visited_nodes:

            x, y = self.grid_to_world(
                gx,
                gy
            )

            p = Point()

            p.x = x
            p.y = y
            p.z = 0.08

            visited_marker.points.append(
                p
            )

        markers.markers.append(
            visited_marker
        )

        # =================================================
        # 4 最终路径
        # =================================================

        path_marker = Marker()

        path_marker.header.frame_id = 'map'
        path_marker.header.stamp = now

        path_marker.ns = 'path'
        path_marker.id = 3

        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD

        path_marker.scale.x = 0.12

        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.2
        path_marker.color.a = 1.0

        for x, y in self.path:

            p = Point()

            p.x = x
            p.y = y
            p.z = 0.15

            path_marker.points.append(
                p
            )

        markers.markers.append(
            path_marker
        )

        # =================================================
        # 5 起点
        # =================================================

        start_marker = Marker()

        start_marker.header.frame_id = 'map'
        start_marker.header.stamp = now

        start_marker.ns = 'start'
        start_marker.id = 4

        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD

        start_marker.pose.position.x = (
            self.start_world[0]
        )

        start_marker.pose.position.y = (
            self.start_world[1]
        )

        start_marker.pose.position.z = 0.3

        start_marker.pose.orientation.w = 1.0

        start_marker.scale.x = 0.5
        start_marker.scale.y = 0.5
        start_marker.scale.z = 0.5

        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0

        markers.markers.append(
            start_marker
        )

        # =================================================
        # 6 终点
        # =================================================

        goal_marker = Marker()

        goal_marker.header.frame_id = 'map'
        goal_marker.header.stamp = now

        goal_marker.ns = 'goal'
        goal_marker.id = 5

        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD

        goal_marker.pose.position.x = (
            self.goal_world[0]
        )

        goal_marker.pose.position.y = (
            self.goal_world[1]
        )

        goal_marker.pose.position.z = 0.3

        goal_marker.pose.orientation.w = 1.0

        goal_marker.scale.x = 0.5
        goal_marker.scale.y = 0.5
        goal_marker.scale.z = 0.5

        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0

        markers.markers.append(
            goal_marker
        )

        self.marker_pub.publish(
            markers
        )


def main(args=None):

    rclpy.init(args=args)

    node = AStarNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
