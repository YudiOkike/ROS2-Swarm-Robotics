import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.publisher_ = self.create_publisher(String, 'task_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_task)  # Publish every second
        self.task_id = 0

    def publish_task(self):
        task = f"Task {self.task_id}: Navigate to point {self.task_id}"
        msg = String()
        msg.data = task
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}'")
        self.task_id += 1


def main(args=None):
    rclpy.init(args=args)
    master_node = MasterNode()
    rclpy.spin(master_node)
    master_node.destroy_node()
    rclpy.shutdown()
