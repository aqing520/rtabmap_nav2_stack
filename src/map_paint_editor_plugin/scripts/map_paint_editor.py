#!/usr/bin/env python3

import signal
import sys

from PyQt5 import QtCore, QtWidgets
import rclpy

from map_editor.node import MapPaintEditorNode


def main():
    app = QtWidgets.QApplication(sys.argv)
    rclpy.init()
    node = MapPaintEditorNode()
    signal.signal(signal.SIGINT, lambda _signum, _frame: app.quit())

    def spin_ros_once():
        if not rclpy.ok():
            app.quit()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except KeyboardInterrupt:
            app.quit()
        except Exception:
            if not rclpy.ok():
                app.quit()
                return
            raise

    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(spin_ros_once)
    spin_timer.start(10)
    try:
        app.exec_()
    finally:
        spin_timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
