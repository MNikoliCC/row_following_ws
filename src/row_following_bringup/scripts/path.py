import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher_node')
        self.path_publisher = self.create_publisher(Path, '/path', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_path)
        self.get_logger().info("Object created!")

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'base_link'
        
        radius = 2
        step = 1

        for angle in range(-90, 90, step):
            angle_rad = math.radians(angle)
            x = radius * math.cos(angle_rad)
            y = radius * math.sin(angle_rad)

            # Add multiple waypoints as PoseStamped messages
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'base_link'
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y + radius
            waypoint.pose.orientation.w = 1.0
            path_msg.poses.append(waypoint)

        self.path_publisher.publish(path_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     path_publisher = PathPublisher()
#     rclpy.spin(path_publisher)
#     path_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
