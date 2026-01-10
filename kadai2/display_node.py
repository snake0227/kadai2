import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class System_Display(Node):

    def __init__(self):
        super().__init__('system_display')
        self.subscription = self.create_subscription(
            String,
            'system_info',
            self.listener_callback,
            10)
        self.get_logger().info('Storage Display Node has started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'RECEIVED: {msg.data}')


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
