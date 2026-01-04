# SPDX-FileCopyrightText: 2025 Daichi Utsugi
# SPDX-License-Identifier: BSD-3-Clause

import os

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
        print('a')


def main():
    rclpy.init()
    node = System_Monitor('.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
