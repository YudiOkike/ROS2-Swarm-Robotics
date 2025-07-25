import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import networkx as nx
import time


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
        self.graph = self.create_graph()

    def create_graph(self):
        # Create a 10x10 grid graph
        graph = nx.grid_2d_graph(10, 10)
        for u, v in graph.edges:
            graph.edges[u, v]['weight'] = 1  # Assign uniform weights

        # Add some initial obstacles by removing edges
        self.add_obstacle(graph, (5, 5), (5, 6))  # Example obstacle
        self.add_obstacle(graph, (7, 8), (7, 9))  # Example obstacle
        return graph

    def add_obstacle(self, graph, node1, node2):
        if graph.has_edge(node1, node2):
            graph.remove_edge(node1, node2)
            self.get_logger().info(f"Added obstacle between {node1} and {node2}")

    def handle_task(self, msg):
        self.get_logger().info(f"Received: '{msg.data}'")
        try:
            start_time = time.time()  # Start timing
            task_id = int(msg.data.split()[-1])  # Extract task ID
            start = (0, 0)  # Starting point
            goal = (task_id % 10, task_id // 10)  # Map task ID to grid point

            # Check if the goal is within the graph
            if goal not in self.graph:
                self.get_logger().error(f"Invalid goal point {goal}. Not in graph!")
                return

            # Simulate a dynamic obstacle
            if task_id == 10:  # Example condition to add a dynamic obstacle
                self.add_obstacle(self.graph, (2, 1), (2, 2))  # Add a new obstacle

            self.get_logger().info(f"Calculating path from {start} to {goal}...")

            # Compute the shortest path using A*
            path = nx.astar_path(self.graph, start, goal, heuristic=self.manhattan_distance, weight='weight')
            end_time = time.time()  # End timing
            self.get_logger().info(f"Path to goal {goal}: {path}")
            self.get_logger().info(f"Time taken for task {task_id}: {end_time - start_time:.2f} seconds")
        except nx.NetworkXNoPath:
            self.get_logger().error(f"No path found to goal!")
        except ValueError:
            self.get_logger().error(f"Invalid task message format: {msg.data}")

    @staticmethod
    def manhattan_distance(a, b):
        # Heuristic function: Manhattan distance
        return abs(a[0] - b[0]) + abs(a[1] - b[1])


def main(args=None):
    rclpy.init(args=args)
    slave_node = SlaveNode()
    rclpy.spin(slave_node)
    slave_node.destroy_node()
    rclpy.shutdown()
