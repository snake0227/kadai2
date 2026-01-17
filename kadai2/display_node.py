# SPDX-FileCopyrightText: 2025 Daichi Utsugi
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class System_Display(Node):
    def __init__(self):
        super().__init__('system_display')
        self.cpu_sub = self.create_subscription(Float32, 'cpu_usage', self.cpu_cb, 10)
        self.memory_sub = self.create_subscription(Float32, 'memory_usage', self.memory_cb, 10)
        self.disk_sub = self.create_subscription(Float32, 'disk_usage', self.disk_cb, 10)
        self.disk_gb_sub = self.create_subscription(Float32, 'disk_gb', self.disk_gb_cb, 10)
        self.get_logger().info('Storage Display Node has started.')

    def cpu_cb(self, msg):
        self.get_logger().info(f'[RECEIVED] CPU Usage: {msg.data}%')

    def memory_cb(self, msg):
        self.get_logger().info(f'[RECEIVED] Memory Usage: {msg.data}%')

    def disk_cb(self, msg):
        self.get_logger().info(f'[RECEIVED] Disk Usage: {msg.data:.1f}%')

    def disk_gb_cb(self, msg):
        self.get_logger().info(f'[RECEIVED] Disk Raw Usage: {msg.data} GB')    

def main():
    rclpy.init()
    node = System_Display()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
