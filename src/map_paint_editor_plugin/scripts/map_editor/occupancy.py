import math
from dataclasses import dataclass

import numpy as np
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid


UNKNOWN = -1
FREE = 0
OCCUPIED = 100


@dataclass(frozen=True)
class MapFiles:
    yaml_path: str
    image_path: str


def msg_to_grid(msg: OccupancyGrid) -> np.ndarray:
    return np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width)).copy()


def grid_to_msg(template: OccupancyGrid, grid: np.ndarray, stamp) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header = template.header
    msg.header.stamp = stamp
    msg.info = template.info
    msg.data = grid.astype(np.int8).reshape(-1).tolist()
    return msg


def grid_to_pgm_image(grid: np.ndarray) -> np.ndarray:
    image = np.full(grid.shape, 205, dtype=np.uint8)
    image[grid >= 65] = 0
    image[(grid >= 0) & (grid <= 25)] = 254
    return np.flipud(image)


def grid_to_display_image(grid: np.ndarray) -> np.ndarray:
    image = np.full(grid.shape, 127, dtype=np.uint8)
    image[grid >= 65] = 0
    image[(grid >= 0) & (grid <= 25)] = 255
    return np.flipud(image)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q
