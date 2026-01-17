# SPDX-FileCopyrightText: 2025 Daichi Utsugi
# SPDX-License-Identifier: BSD-3-Clause

import os
import shutil
import sys
import psutil

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class System_Monitor(Node):

    def __init__(self, target_path):
        super().__init__('system_monitor')
        self.disk_pub = self.create_publisher(Float32, 'disk_usage', 10)
        self.cpu_pub = self.create_publisher(Float32, 'cpu_usage', 10)
        self.memory_pub = self.create_publisher(Float32, 'memory_usage', 10)
        self.disk_gb_pub = self.create_publisher(Float32, 'disk_gb', 10)

        self.target = os.path.abspath(target_path)
        self.timer = self.create_timer(2.0, self.cb)
        self.get_logger().info(f'Monitoring path: {self.target}')
    def cb(self):
        try:
            total, used, _ = shutil.disk_usage(self.target)
            using = used / 1000 ** 3
            disk_p = (used / total) * 100
            cpu_p = psutil.cpu_percent()
            memory_p = psutil.virtual_memory().percent
            
            
            disk_msg = Float32()
            disk_msg.data = disk_p
            self.disk_pub.publish(disk_msg)
            
            cpu_msg = Float32()
            cpu_msg.data = cpu_p
            self.cpu_pub.publish(cpu_msg)
            
            memory_msg = Float32()
            memory_msg.data = memory_p
            self.memory_pub.publish(memory_msg)

            disk_gb_msg = Float32()
            disk_gb_msg.data = float(using)
            self.disk_gb_pub.publish(disk_gb_msg)
            
            self.get_logger().info(f'Published metrics - CPU: {cpu_p}%, Memory: {memory_p}%, Disk: {using}GB,{disk_p:.2f}%')

        
        except Exception as e:
            self.get_logger().error(f'Failed to get disk usage: {e}')


def main():
    rclpy.init()
    if len(sys.argv) < 2:
        print("Usage: system_monitor <path>")
        sys.exit(1)
    
    target = sys.argv[1]
    if not os.path.exists(target):
        print(f"Error: Path {target} does not exist")
        sys.exit(1)

    node = System_Monitor(target)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
