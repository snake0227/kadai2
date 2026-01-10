# SPDX-FileCopyrightText: 2025 Daichi Utsugi
# SPDX-License-Identifier: BSD-3-Clause

import os
import shutil
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class System_Monitor(Node):

    def __init__(self, target_path):
        super().__init__('system_monitor')
        self.pub = self.create_publisher(String, 'system_info', 10)
        self.target = os.path.abspath(target_path)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        try:
            total, used, free = shutil.disk_usage(self.target)
            total_gb = total / (2**30)
            used_gb = used / (2**30)
            percent = (used / total) * 100
            msg_content = (
                f'Path: {self.target} | '
                f'Total: {total_gb:.1f}GB | '
                f'Used: {used_gb:.1f}GB ({percent:.1f}%)'
            )
            msg = String()
            msg.data = msg_content
            self.pub.publish(msg)
            self.get_logger().info(f'Publishing: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Failed to get disk usage: {e}')


def main():
    rclpy.init()
    target = sys.argv[1] if len(sys.argv) > 1 else '.'
    node = System_Monitor(target)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
