import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import ClearEntireCostmap
from PyQt5 import QtWidgets
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .canvas import MapEditorCanvas
from .occupancy import grid_to_msg, msg_to_grid
from .storage import MapStorage


class MapPaintEditorNode(Node):
    def __init__(self):
        super().__init__('map_paint_editor')

        self.load_yaml_path = self.declare_parameter(
            'load_yaml_path',
            '/data/maps/site_a/map.yaml',
        ).value
        self.input_topic = self.declare_parameter('input_topic', '').value
        self.output_topic = self.declare_parameter('output_topic', '/map_edited').value
        self.save_yaml_path = self.declare_parameter(
            'save_yaml_path',
            '/data/maps/site_a/map.yaml',
        ).value
        self.brush_radius = int(self.declare_parameter('brush_radius_cells', 5).value)
        self.max_window_size = int(self.declare_parameter('max_window_size', 1000).value)
        self.initial_zoom = float(self.declare_parameter('initial_zoom', 1.0).value)
        self.max_zoom = float(self.declare_parameter('max_zoom', 8.0).value)
        self.window_name = str(self.declare_parameter('window_name', 'Map Editor').value)
        self.clear_global_costmap_on_publish = bool(
            self.declare_parameter(
                'clear_global_costmap_on_publish',
                self.output_topic == '/map',
            ).value
        )
        self.clear_global_costmap_service = str(
            self.declare_parameter(
                'clear_global_costmap_service',
                '/global_costmap/clear_entirely_global_costmap',
            ).value
        )
        self.costmap_clear_min_interval_sec = float(
            self.declare_parameter('costmap_clear_min_interval_sec', 0.75).value
        )
        self.reissue_goal_on_publish = bool(
            self.declare_parameter(
                'reissue_goal_on_publish',
                self.output_topic == '/map',
            ).value
        )
        self.goal_topic = str(self.declare_parameter('goal_topic', '/goal_pose').value)
        self.goal_reissue_delay_sec = float(
            self.declare_parameter('goal_reissue_delay_sec', 0.35).value
        )
        self.goal_reissue_min_interval_sec = float(
            self.declare_parameter('goal_reissue_min_interval_sec', 1.0).value
        )

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(OccupancyGrid, self.output_topic, qos)
        self.subscription = None
        if self.input_topic:
            self.subscription = self.create_subscription(
                OccupancyGrid,
                self.input_topic,
                self.map_callback,
                qos,
            )
        self.goal_subscription = None
        self.goal_publisher = None
        if self.reissue_goal_on_publish:
            self.goal_subscription = self.create_subscription(
                PoseStamped,
                self.goal_topic,
                self.goal_callback,
                10,
            )
            self.goal_publisher = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.storage = MapStorage(self.save_yaml_path)
        self.map_msg = None
        self.grid = None
        self.dirty = False
        self.last_publish_time = self.get_clock().now()
        self.last_costmap_clear_time = None
        self.last_goal_msg = None
        self.last_goal_reissue_time = None
        self.goal_reissue_timer = None
        self.clear_global_costmap_client = None
        if self.clear_global_costmap_on_publish:
            self.clear_global_costmap_client = self.create_client(
                ClearEntireCostmap,
                self.clear_global_costmap_service,
            )

        self.canvas = MapEditorCanvas(
            window_name=self.window_name,
            max_window_size=self.max_window_size,
            brush_radius=self.brush_radius,
            initial_zoom=self.initial_zoom,
            max_zoom=self.max_zoom,
            on_paint=self.paint_at_display_pixel,
            on_publish=self.publish_map,
            on_save=self.save_map,
            on_quit=self.shutdown,
        )
        self.timer = self.create_timer(0.05, self.ui_timer)
        self.load_saved_map()

        self.get_logger().info(
            'Map editor ready: load %s, subscribe %s, publish %s, save button overwrites %s, clear_global_costmap=%s, reissue_goal=%s'
            % (
                self.load_yaml_path,
                self.input_topic or '<disabled>',
                self.output_topic,
                self.save_yaml_path,
                self.clear_global_costmap_on_publish,
                self.reissue_goal_on_publish,
            )
        )

    def load_saved_map(self) -> None:
        if not self.load_yaml_path:
            return
        try:
            msg = MapStorage(self.load_yaml_path).load()
        except RuntimeError as exc:
            self.get_logger().warn(str(exc))
            return
        self.load_map_msg(msg, self.load_yaml_path)

    def map_callback(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or msg.info.height == 0:
            return
        if self.grid is not None and self.dirty:
            return

        self.load_map_msg(msg, self.input_topic)

    def load_map_msg(self, msg: OccupancyGrid, source: str) -> None:
        self.map_msg = msg
        self.grid = msg_to_grid(msg)
        self.canvas.update_scale(msg.info.width, msg.info.height)
        self.get_logger().info(
            'Loaded map %dx%d resolution=%.3f from %s'
            % (msg.info.width, msg.info.height, msg.info.resolution, source)
        )
        self.publish_map()

    def paint_at_display_pixel(self, display_x: int, display_y: int, value: int) -> None:
        if self.grid is None:
            return

        height, width = self.grid.shape
        grid_xy = self.canvas.display_to_grid(display_x, display_y, height, width)
        if grid_xy is None:
            return
        gx, gy = grid_xy

        import numpy as np
        yy, xx = np.ogrid[:height, :width]
        mask = (xx - gx) ** 2 + (yy - gy) ** 2 <= self.canvas.brush_radius ** 2
        self.grid[mask] = value
        self.dirty = True

        now = self.get_clock().now()
        if (now - self.last_publish_time).nanoseconds > 150_000_000:
            self.publish_map()

    def ui_timer(self) -> None:
        if self.grid is None:
            waiting_source = self.load_yaml_path or self.input_topic or 'saved map file'
            self.canvas.show_waiting(waiting_source)
        else:
            self.canvas.show_grid(self.grid, self.dirty, self.save_yaml_path)
        self.canvas.wait_key()

    def publish_map(self) -> None:
        if self.map_msg is None or self.grid is None:
            return
        msg = grid_to_msg(self.map_msg, self.grid, self.get_clock().now().to_msg())
        self.publisher.publish(msg)
        self.last_publish_time = self.get_clock().now()
        self.request_global_costmap_clear()
        self.request_goal_reissue()

    def save_map(self) -> None:
        if self.map_msg is None or self.grid is None:
            self.get_logger().warn('No map loaded yet.')
            return
        files = self.storage.save(self.grid, self.map_msg.info)
        self.dirty = False
        self.publish_map()
        self.get_logger().info('Overwrote edited map: %s and %s' % (files.yaml_path, files.image_path))

    def shutdown(self) -> None:
        QtWidgets.QApplication.quit()

    def goal_callback(self, msg: PoseStamped) -> None:
        if msg.header.frame_id:
            self.last_goal_msg = msg

    def request_global_costmap_clear(self) -> None:
        if not self.clear_global_costmap_client:
            return

        now = self.get_clock().now()
        if self.last_costmap_clear_time is not None:
            elapsed_sec = (now - self.last_costmap_clear_time).nanoseconds / 1_000_000_000.0
            if elapsed_sec < self.costmap_clear_min_interval_sec:
                return

        if not self.clear_global_costmap_client.service_is_ready():
            return

        self.last_costmap_clear_time = now
        self.clear_global_costmap_client.call_async(ClearEntireCostmap.Request())

    def request_goal_reissue(self) -> None:
        if not self.goal_publisher or self.last_goal_msg is None:
            return

        now = self.get_clock().now()
        if self.last_goal_reissue_time is not None:
            elapsed_sec = (now - self.last_goal_reissue_time).nanoseconds / 1_000_000_000.0
            if elapsed_sec < self.goal_reissue_min_interval_sec:
                return

        if self.goal_reissue_timer is not None:
            return

        self.goal_reissue_timer = self.create_timer(
            self.goal_reissue_delay_sec,
            self.reissue_last_goal,
        )

    def reissue_last_goal(self) -> None:
        if self.goal_reissue_timer is not None:
            self.destroy_timer(self.goal_reissue_timer)
            self.goal_reissue_timer = None

        if self.last_goal_msg is None or self.goal_publisher is None:
            return

        goal = PoseStamped()
        goal.header = self.last_goal_msg.header
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = self.last_goal_msg.pose
        self.goal_publisher.publish(goal)
        self.last_goal_reissue_time = self.get_clock().now()
        self.get_logger().info('Reissued latest Nav2 goal after edited map publish.')
