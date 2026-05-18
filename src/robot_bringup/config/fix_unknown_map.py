#!/usr/bin/env python3
"""
Fill unknown (gray) cells in a ROS2 occupancy grid map PGM file.

Usage:
  # Step 1 - save the live /map topic to a file:
  ros2 run nav2_map_server map_saver_cli -f /tmp/parking_map

  # Step 2 - run this script:
  python3 fix_unknown_map.py /tmp/parking_map.pgm /tmp/parking_map_fixed.pgm

  # Step 3 - copy the fixed map alongside the original yaml:
  cp /tmp/parking_map.yaml /tmp/parking_map_fixed.yaml
  sed -i 's/parking_map.pgm/parking_map_fixed.pgm/' /tmp/parking_map_fixed.yaml
"""

import sys
import cv2
import numpy as np


# In ROS2 nav2 map_saver output PGMs:
#   254 = free (white)
#   0   = occupied (black)
#   205 = unknown (gray)
FREE_VALUE     = 254
OCCUPIED_VALUE = 0
UNKNOWN_VALUE  = 205

# Pixels within this range are treated as unknown and will be filled.
UNKNOWN_LOW  = 170
UNKNOWN_HIGH = 230


def fill_unknown(src_path: str, dst_path: str) -> None:
    img = cv2.imread(src_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"Cannot read image: {src_path}")

    original = img.copy()

    # Mask of unknown cells
    unknown_mask = (img >= UNKNOWN_LOW) & (img <= UNKNOWN_HIGH)

    # Mask of occupied cells (keep them as reference seeds for inpainting)
    occupied_mask = img < 50

    print(f"Unknown cells  : {unknown_mask.sum()}")
    print(f"Occupied cells : {occupied_mask.sum()}")
    print(f"Free cells     : {(img > UNKNOWN_HIGH).sum()}")

    # --- Strategy: fill unknown cells that are NOT adjacent to occupied cells
    # with FREE, then inpaint the rest for a smooth result.

    # Dilate occupied mask slightly so we don't accidentally free cells right
    # next to walls.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    occupied_dilated = cv2.dilate(occupied_mask.astype(np.uint8), kernel, iterations=2)

    # Unknown cells that are safely away from obstacles → free
    safe_unknown = unknown_mask & (occupied_dilated == 0)
    img[safe_unknown] = FREE_VALUE

    # Remaining unknown cells (near obstacles) → inpaint for smooth transition
    remaining_unknown = (img >= UNKNOWN_LOW) & (img <= UNKNOWN_HIGH)
    if remaining_unknown.sum() > 0:
        inpaint_mask = remaining_unknown.astype(np.uint8) * 255
        img = cv2.inpaint(img, inpaint_mask, inpaintRadius=5, flags=cv2.INPAINT_TELEA)

    cv2.imwrite(dst_path, img)

    filled = unknown_mask.sum() - ((img >= UNKNOWN_LOW) & (img <= UNKNOWN_HIGH)).sum()
    print(f"Filled {filled} unknown cells → {dst_path}")


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    fill_unknown(sys.argv[1], sys.argv[2])
