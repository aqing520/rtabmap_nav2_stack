#!/usr/bin/env python3

import os
import threading
import time
from typing import Optional

import cv2
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


def _qos_from_int(value: int) -> QoSProfile:
    if value == 1:
        reliability = ReliabilityPolicy.RELIABLE
    elif value == 2:
        reliability = ReliabilityPolicy.BEST_EFFORT
    else:
        reliability = ReliabilityPolicy.SYSTEM_DEFAULT
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=reliability)


class RtspCameraBridge(Node):
    def __init__(self) -> None:
        super().__init__('rtsp_camera_bridge')

        self.declare_parameter('rtsp_url', '')
        self.declare_parameter('gstreamer_pipeline', '')
        self.declare_parameter('backend', 'ffmpeg')
        self.declare_parameter('rtsp_transport', 'tcp')
        self.declare_parameter('ffmpeg_options', '')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('reconnect_delay_sec', 2.0)
        self.declare_parameter('target_fps', 0.0)
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)
        self.declare_parameter('qos_image', 1)
        self.declare_parameter('qos_camera_info', 1)
        self.declare_parameter('force_mono', False)
        self.declare_parameter('focal_length_px', 0.0)

        image_qos = _qos_from_int(self.get_parameter('qos_image').get_parameter_value().integer_value)
        info_qos = _qos_from_int(self.get_parameter('qos_camera_info').get_parameter_value().integer_value)
        self.image_pub = self.create_publisher(Image, 'image_raw', image_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', info_qos)

        self._capture: Optional[cv2.VideoCapture] = None
        self._running = True
        self._camera_info = self._load_camera_info()
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    def destroy_node(self) -> bool:
        self._running = False
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        if self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)
        return super().destroy_node()

    def _load_camera_info(self) -> CameraInfo:
        path = self.get_parameter('camera_info_url').get_parameter_value().string_value.strip()
        msg = CameraInfo()
        msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        msg.distortion_model = 'plumb_bob'

        if not path:
            self.get_logger().warn(
                'camera_info_url is empty, using approximate intrinsics. '
                'Provide a calibration yaml for usable metric results.')
            return msg

        if path.startswith('file://'):
            path = path[7:]

        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)

        msg.width = int(data.get('image_width', 0))
        msg.height = int(data.get('image_height', 0))
        msg.distortion_model = data.get('distortion_model', 'plumb_bob')
        msg.d = list(data.get('distortion_coefficients', {}).get('data', []))
        msg.k = list(data.get('camera_matrix', {}).get('data', [0.0] * 9))
        msg.r = list(data.get('rectification_matrix', {}).get('data', [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]))
        msg.p = list(data.get('projection_matrix', {}).get('data', [0.0] * 12))
        self.get_logger().info('Loaded camera calibration from %s' % path)
        return msg

    def _ensure_camera_info(self, width: int, height: int) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        if self._camera_info.width > 0 and self._camera_info.height > 0:
            msg = self._camera_info
            msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
            return msg

        focal = self.get_parameter('focal_length_px').get_parameter_value().double_value
        if focal <= 0.0:
            focal = float(max(width, height))
        cx = float(width) / 2.0
        cy = float(height) / 2.0

        msg.width = width
        msg.height = height
        msg.k = [
            focal, 0.0, cx,
            0.0, focal, cy,
            0.0, 0.0, 1.0,
        ]
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            focal, 0.0, cx, 0.0,
            0.0, focal, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return msg

    def _open_capture(self) -> bool:
        backend = self.get_parameter('backend').get_parameter_value().string_value.lower()
        rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value.strip()
        pipeline = self.get_parameter('gstreamer_pipeline').get_parameter_value().string_value.strip()
        source = pipeline if pipeline else rtsp_url

        if not source:
            self.get_logger().error('rtsp_url is empty and gstreamer_pipeline is empty.')
            return False

        ffmpeg_options = self.get_parameter('ffmpeg_options').get_parameter_value().string_value.strip()
        rtsp_transport = self.get_parameter('rtsp_transport').get_parameter_value().string_value.strip()
        options = []
        if rtsp_transport:
            options.append('rtsp_transport;%s' % rtsp_transport)
        if ffmpeg_options:
            options.append(ffmpeg_options)
        if options:
            os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = '|'.join(options)

        api_preference = cv2.CAP_ANY
        if backend == 'ffmpeg':
            api_preference = cv2.CAP_FFMPEG
        elif backend == 'gstreamer':
            api_preference = cv2.CAP_GSTREAMER

        self._capture = cv2.VideoCapture(source, api_preference)
        if not self._capture.isOpened():
            self.get_logger().error('Failed to open video source: %s' % source)
            self._capture.release()
            self._capture = None
            return False

        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        target_fps = self.get_parameter('target_fps').get_parameter_value().double_value
        if width > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if target_fps > 0.0:
            self._capture.set(cv2.CAP_PROP_FPS, target_fps)

        self.get_logger().info('Opened stream with backend=%s source=%s' % (backend, source))
        return True

    def _capture_loop(self) -> None:
        reconnect_delay = self.get_parameter('reconnect_delay_sec').get_parameter_value().double_value
        force_mono = self.get_parameter('force_mono').get_parameter_value().bool_value
        target_fps = self.get_parameter('target_fps').get_parameter_value().double_value
        frame_period = 0.0 if target_fps <= 0.0 else 1.0 / target_fps
        last_pub = 0.0

        while self._running and rclpy.ok():
            if self._capture is None and not self._open_capture():
                time.sleep(reconnect_delay)
                continue

            assert self._capture is not None
            ok, frame = self._capture.read()
            if not ok or frame is None:
                self.get_logger().warn('Capture read failed, reconnecting...')
                self._capture.release()
                self._capture = None
                time.sleep(reconnect_delay)
                continue

            if force_mono and len(frame.shape) == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            now = time.time()
            if frame_period > 0.0 and now - last_pub < frame_period:
                time.sleep(min(frame_period / 2.0, 0.01))
                continue
            last_pub = now

            stamp = self.get_clock().now().to_msg()
            frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

            image_msg = Image()
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = frame_id
            image_msg.height = int(frame.shape[0])
            image_msg.width = int(frame.shape[1])
            image_msg.encoding = 'mono8' if len(frame.shape) == 2 else 'bgr8'
            image_msg.is_bigendian = False
            image_msg.step = int(frame.strides[0])
            image_msg.data = frame.tobytes()
            self.image_pub.publish(image_msg)

            camera_info = self._ensure_camera_info(image_msg.width, image_msg.height)
            camera_info.header.stamp = stamp
            camera_info.header.frame_id = frame_id
            camera_info.width = image_msg.width
            camera_info.height = image_msg.height
            self.camera_info_pub.publish(camera_info)


def main() -> None:
    rclpy.init()
    node = RtspCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
