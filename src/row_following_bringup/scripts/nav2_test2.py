#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath

# Initialize ROS 2
rclpy.init()

# Create a node
node = rclpy.create_node('follow_path_client')

# Initialize an action client for the FollowPath action
action_client = ActionClient(node, FollowPath, 'follow_path')

# Wait until the server becomes available
node.get_logger().info('Waiting for the FollowPath action server...')
action_client.wait_for_server()
node.get_logger().info('FollowPath action server is ready.')

# Create a FollowPath goal message
goal_msg = FollowPath.Goal()

# Define waypoints in the 'map' frame
waypoint1 = PoseStamped()
waypoint1.header.frame_id = 'odom'
waypoint1.pose.position.x = 0.0
waypoint1.pose.position.y = 0.0
waypoint1.pose.orientation.w = 1.0

waypoint2 = PoseStamped()
waypoint2.header.frame_id = 'odom'
waypoint2.pose.position.x = 0.0
waypoint2.pose.position.y = 0.0
waypoint2.pose.orientation.w = 1.0

# Add the waypoints to the goal
goal_msg.path.poses = [waypoint1, waypoint2]

# Handle goal response and results
def goal_response_callback(future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().info('Goal rejected')
        return

    node.get_logger().info('Goal accepted')
    # goal_handle.add_feedback_callback(feedback_callback)
    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(result_callback)

def feedback_callback(feedback_msg):
    # Process and print the feedback from the server
    node.get_logger().info(f'Current progress: {feedback_msg.feedback}')

def result_callback(future):
    result = future.result().result
    node.get_logger().info(f'FollowPath Result: {result.result}')

# Send the goal asynchronously
send_goal_future = action_client.send_goal_async(goal_msg)
send_goal_future.add_done_callback(goal_response_callback)

# Spin the node to keep it alive
rclpy.spin(node)

# Shutdown the node when done
node.destroy_node()
rclpy.shutdown()
