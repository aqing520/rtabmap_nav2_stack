from datetime import datetime
from pathlib import Path

import rclpy
from PyQt5 import QtWidgets
from rclpy.node import Node

from .canvas import MapEditorCanvas
from .occupancy import msg_to_grid
from .storage import MapStorage


class MapPaintEditorNode(Node):
    def __init__(self):
        super().__init__('map_paint_editor')

        self.load_yaml_path = self.declare_parameter(
            'load_yaml_path',
            '/data/maps/site_a/map.yaml',
        ).value
        self.save_yaml_path = self.declare_parameter(
            'save_yaml_path',
            '/data/maps/site_a/map.yaml',
        ).value
        self.brush_radius = int(self.declare_parameter('brush_radius_cells', 5).value)
        self.max_window_size = int(self.declare_parameter('max_window_size', 1000).value)
        self.initial_zoom = float(self.declare_parameter('initial_zoom', 1.0).value)
        self.max_zoom = float(self.declare_parameter('max_zoom', 8.0).value)
        self.window_name = str(
            self.declare_parameter('window_name', 'Offline Map Editor').value
        )

        self.storage = MapStorage(self.save_yaml_path)
        self.map_msg = None
        self.grid = None
        self.dirty = False

        self.canvas = MapEditorCanvas(
            window_name=self.window_name,
            max_window_size=self.max_window_size,
            brush_radius=self.brush_radius,
            initial_zoom=self.initial_zoom,
            max_zoom=self.max_zoom,
            on_paint=self.paint_at_display_pixel,
            on_save=self.save_map,
            on_quit=self.shutdown,
        )
        self.timer = self.create_timer(0.05, self.ui_timer)
        self.load_saved_map()

        self.get_logger().info(
            'Offline map editor ready: load %s, save button overwrites %s'
            % (
                self.load_yaml_path,
                self.save_yaml_path,
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

    def load_map_msg(self, msg, source: str) -> None:
        self.map_msg = msg
        self.grid = msg_to_grid(msg)
        self.canvas.update_scale(msg.info.width, msg.info.height)
        source_path = Path(source).expanduser()
        source_modified = 'unknown'
        if source_path.exists():
            source_modified = datetime.fromtimestamp(
                source_path.stat().st_mtime
            ).astimezone().isoformat(timespec='seconds')
        self.get_logger().info(
            'Loaded map %dx%d resolution=%.3f from %s (modified %s)'
            % (
                msg.info.width,
                msg.info.height,
                msg.info.resolution,
                source,
                source_modified,
            )
        )

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

    def ui_timer(self) -> None:
        if self.grid is None:
            waiting_source = self.load_yaml_path or 'saved map file'
            self.canvas.show_waiting(waiting_source)
        else:
            self.canvas.show_grid(self.grid, self.dirty, self.save_yaml_path)
        self.canvas.wait_key()

    def save_map(self) -> None:
        if self.map_msg is None or self.grid is None:
            self.get_logger().warn('No map loaded yet.')
            return
        files = self.storage.save(self.grid, self.map_msg.info)
        self.dirty = False
        self.get_logger().info('Overwrote edited map: %s and %s' % (files.yaml_path, files.image_path))

    def shutdown(self) -> None:
        QtWidgets.QApplication.quit()
