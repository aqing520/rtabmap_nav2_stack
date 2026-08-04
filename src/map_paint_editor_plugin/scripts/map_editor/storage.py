from pathlib import Path

import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid

from .occupancy import MapFiles, grid_to_pgm_image, quaternion_to_yaw, yaw_to_quaternion


DEFAULT_OCCUPIED_THRESH = 0.65
# Unknown cells are stored as grayscale 205. Their occupancy probability is
# (255 - 205) / 255 ~= 0.196078, so the free threshold must stay below that
# value. Otherwise map_server and this editor both decode unknown cells as free.
DEFAULT_FREE_THRESH = 0.196


class MapStorage:
    def __init__(self, yaml_path: str):
        self.yaml_path = Path(yaml_path).expanduser()
        self.image_path = self.yaml_path.with_suffix('.pgm')

    def save(self, grid, map_info) -> MapFiles:
        self.yaml_path.parent.mkdir(parents=True, exist_ok=True)
        image = grid_to_pgm_image(grid)
        if not cv2.imwrite(str(self.image_path), image):
            raise RuntimeError('Cannot write map image: %s' % self.image_path)

        origin = map_info.origin
        yaw = quaternion_to_yaw(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        self.yaml_path.write_text(
            'image: %s\n'
            'mode: trinary\n'
            'resolution: %.10f\n'
            'origin: [%.10f, %.10f, %.10f]\n'
            'negate: 0\n'
            'occupied_thresh: %.3f\n'
            'free_thresh: %.3f\n'
            % (
                self.image_path.name,
                map_info.resolution,
                origin.position.x,
                origin.position.y,
                yaw,
                DEFAULT_OCCUPIED_THRESH,
                DEFAULT_FREE_THRESH,
            ),
            encoding='utf-8',
        )
        return MapFiles(str(self.yaml_path), str(self.image_path))

    def load(self) -> OccupancyGrid:
        values = self._read_yaml()
        image_path = self._resolve_image_path(values.get('image', self.image_path.name))
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError('Cannot read map image: %s' % image_path)

        negate = int(values.get('negate', '0'))
        occupied_thresh = float(
            values.get('occupied_thresh', str(DEFAULT_OCCUPIED_THRESH)))
        free_thresh = float(values.get('free_thresh', str(DEFAULT_FREE_THRESH)))
        resolution = float(values['resolution'])
        origin_values = self._parse_origin(values.get('origin', '[0.0, 0.0, 0.0]'))

        if not 0.0 <= free_thresh < occupied_thresh <= 1.0:
            raise RuntimeError(
                'Invalid map thresholds: expected 0 <= free_thresh < '
                'occupied_thresh <= 1, got free_thresh=%s, occupied_thresh=%s'
                % (free_thresh, occupied_thresh))

        if negate:
            image = 255 - image

        occupancy = (255.0 - image.astype(np.float32)) / 255.0
        grid = np.full(image.shape, -1, dtype=np.int16)
        grid[occupancy > occupied_thresh] = 100
        grid[occupancy < free_thresh] = 0
        grid = np.flipud(grid)

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.width = int(grid.shape[1])
        msg.info.height = int(grid.shape[0])
        msg.info.resolution = resolution
        msg.info.origin.position.x = origin_values[0]
        msg.info.origin.position.y = origin_values[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation = yaw_to_quaternion(origin_values[2])
        msg.data = grid.astype(np.int8).reshape(-1).tolist()
        return msg

    def _read_yaml(self):
        if not self.yaml_path.exists():
            raise RuntimeError('Map yaml does not exist: %s' % self.yaml_path)

        values = {}
        for raw_line in self.yaml_path.read_text(encoding='utf-8').splitlines():
            line = raw_line.split('#', 1)[0].strip()
            if not line or ':' not in line:
                continue
            key, value = line.split(':', 1)
            values[key.strip()] = value.strip().strip('"').strip("'")
        return values

    def _resolve_image_path(self, image_value: str) -> Path:
        image_path = Path(image_value).expanduser()
        if image_path.is_absolute():
            return image_path
        return self.yaml_path.parent / image_path

    @staticmethod
    def _parse_origin(value: str):
        cleaned = value.strip().strip('[]')
        parts = [float(part.strip()) for part in cleaned.split(',') if part.strip()]
        while len(parts) < 3:
            parts.append(0.0)
        return parts[:3]
