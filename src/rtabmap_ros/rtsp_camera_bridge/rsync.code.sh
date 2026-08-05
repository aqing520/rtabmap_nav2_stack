#!/bin/bash

rsync -r . wheeltec@192.168.110.198:rtabmap_nav2_stack/src/rtabmap_ros/rtsp_camera_bridge/

echo "rsync done"
