import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SlaveNode(Node):
    def __init__(self):
        super().__init__('slave_node')
        self.subscription = self.create_subscription(
            String,
            'task_topic',
            self.handle_task,
            10
        )
        self.subscription  # Prevent unused variable warning

    def handle_task(self, msg):
        self.get_logger().info(f"Received: '{msg.data}'")
        # Placeholder for task execution logic


def main(args=None):
    rclpy.init(args=args)
    slave_node = SlaveNode()
    rclpy.spin(slave_node)
    slave_node.destroy_node()
    rclpy.shutdown()
