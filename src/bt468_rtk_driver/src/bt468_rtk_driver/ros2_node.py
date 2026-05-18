from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import String

from .nmea import ParsedSentence, parse_nmea_line
from .serial_reader import SerialStream, candidate_ports


def _navsat_status_from_label(label: str) -> int:
    if label == "没定位":
        return NavSatStatus.STATUS_NO_FIX
    if label == "DGPS":
        return NavSatStatus.STATUS_SBAS_FIX
    if label in {"RTK Float", "RTK Fixed"}:
        return NavSatStatus.STATUS_GBAS_FIX
    return NavSatStatus.STATUS_FIX


def _service_mask() -> int:
    return (
        NavSatStatus.SERVICE_GPS
        | NavSatStatus.SERVICE_GLONASS
        | NavSatStatus.SERVICE_COMPASS
        | NavSatStatus.SERVICE_GALILEO
    )


class BT468RtkNode(Node):
    def __init__(self) -> None:
        super().__init__("bt468_rtk_node")

        self.declare_parameter("port", "auto")
        self.declare_parameter("baud", 38400)
        self.declare_parameter("timeout_sec", 0.2)
        self.declare_parameter("reconnect_delay_sec", 1.0)
        self.declare_parameter("frame_id", "gnss_link")
        self.declare_parameter("log_summary", True)

        self.fix_pub = self.create_publisher(NavSatFix, "fix", 10)
        self.status_pub = self.create_publisher(String, "fix_status", 10)
        self.nmea_pub = self.create_publisher(String, "nmea_sentence", 20)

        self._running = True
        self._worker = threading.Thread(target=self._read_loop, daemon=True)
        self._worker.start()

    def destroy_node(self) -> bool:
        self._running = False
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()

    def _resolve_port(self) -> str:
        port = self.get_parameter("port").get_parameter_value().string_value
        if port != "auto":
            return port
        ports = candidate_ports()
        if not ports:
            raise RuntimeError("No candidate serial ports found.")
        return ports[0]

    def _build_fix_msg(self, parsed: ParsedSentence) -> NavSatFix:
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        msg.status.service = _service_mask()
        label = str(parsed.fields.get("fix_status", "未知"))
        msg.status.status = _navsat_status_from_label(label)
        msg.latitude = float(parsed.fields["lat"])
        msg.longitude = float(parsed.fields["lon"])
        altitude_m = parsed.fields.get("altitude_m")
        msg.altitude = float(altitude_m) if altitude_m is not None else float("nan")
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return msg

    def _publish_gga(self, parsed: ParsedSentence) -> None:
        status_label = str(parsed.fields.get("fix_status", "未知"))
        lat = parsed.fields.get("lat")
        lon = parsed.fields.get("lon")

        status_msg = String()
        status_msg.data = status_label
        self.status_pub.publish(status_msg)

        if lat is not None and lon is not None:
            self.fix_pub.publish(self._build_fix_msg(parsed))

        if self.get_parameter("log_summary").get_parameter_value().bool_value:
            lat_text = "-" if lat is None else f"{float(lat):.9f}"
            lon_text = "-" if lon is None else f"{float(lon):.9f}"
            self.get_logger().info(f"定位状态={status_label} 纬度={lat_text} 经度={lon_text}")

    def _read_loop(self) -> None:
        reconnect_delay = self.get_parameter("reconnect_delay_sec").get_parameter_value().double_value

        while self._running and rclpy.ok():
            try:
                port = self._resolve_port()
                baud = self.get_parameter("baud").get_parameter_value().integer_value
                timeout = self.get_parameter("timeout_sec").get_parameter_value().double_value
                self.get_logger().info(f"Opening GNSS port {port} at {baud} baud")

                stream = SerialStream(port=port, baudrate=baud, timeout=timeout)
                for line in stream.read_lines():
                    if not self._running or not rclpy.ok():
                        return
                    nmea_msg = String()
                    nmea_msg.data = line
                    self.nmea_pub.publish(nmea_msg)

                    parsed_sentence = parse_nmea_line(line)
                    if parsed_sentence is None:
                        continue
                    if parsed_sentence.type == "GGA":
                        self._publish_gga(parsed_sentence)
            except Exception as exc:
                self.get_logger().warn(f"GNSS read loop error: {exc}")
                time.sleep(reconnect_delay)


def main() -> None:
    rclpy.init()
    node = BT468RtkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except AttributeError:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
