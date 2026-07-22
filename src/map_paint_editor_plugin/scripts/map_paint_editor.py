#!/usr/bin/env python3

import sys

from PyQt5 import QtCore, QtWidgets
import rclpy

from map_editor.node import MapPaintEditorNode


def main():
    app = QtWidgets.QApplication(sys.argv)
    rclpy.init()
    node = MapPaintEditorNode()
    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
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
