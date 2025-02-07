import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.ndimage import label, center_of_mass

class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_exploration')
        
        # Subscribers to global and local costmaps
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.global_costmap_callback, 10)
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, 10)
        
        # Publisher for exploration goal
        self.goal_pub = self.create_publisher(PoseStamped, '/exploration_goal', 10)
        
        self.global_costmap = None
        self.local_costmap = None

        # self.timer = self.create_timer(0.1, self.process_frontiers)
        
    def global_costmap_callback(self, msg):
        self.global_costmap = msg
        self.process_frontiers()

    def local_costmap_callback(self, msg):
        self.local_costmap = msg
        self.process_frontiers()
    
    def process_frontiers(self):
        # self.get_logger().info("Process running")
        if self.global_costmap is None or self.local_costmap is None:
            self.get_logger().info("No Costmaps or global maps found.")
            return
        
        # Convert occupancy grid to numpy array
        width = self.global_costmap.info.width
        height = self.global_costmap.info.height
        resolution = self.global_costmap.info.resolution
        origin = self.global_costmap.info.origin.position
        
        grid = np.array(self.global_costmap.data, dtype=np.int8).reshape(height, width)
        
        # Define frontier as transition from unknown (-1) to free (0)
        frontier_mask = (grid == -1) & (
            (np.roll(grid, 1, axis=0) == 0) | (np.roll(grid, -1, axis=0) == 0) | 
            (np.roll(grid, 1, axis=1) == 0) | (np.roll(grid, -1, axis=1) == 0)
        )
        
        # Label frontiers and find centroids
        labeled_frontiers, num_frontiers = label(frontier_mask)
        if num_frontiers == 0:
            self.get_logger().info("No frontiers found.")
            return
        
        centroids = center_of_mass(frontier_mask, labeled_frontiers, range(1, num_frontiers + 1))
        
        # Convert centroids to world coordinates
        exploration_goals = []
        for cy, cx in centroids:
            wx = origin.x + cx * resolution
            wy = origin.y + cy * resolution
            exploration_goals.append((wx, wy))
        
        # Select nearest frontier
        if exploration_goals:
            target_x, target_y = exploration_goals[0]
            self.publish_goal(target_x, target_y)
    
    def publish_goal(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0  # No rotation
        
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Published exploration goal: ({x}, {y})")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
