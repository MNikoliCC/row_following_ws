#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped, Transform, Pose
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
import ros2_numpy as rnp
import numpy as np
from transforms3d.euler import euler2mat

import math

class FollowPathClient(Node):
    def __init__(self):
        super().__init__('follow_path_client')
        self.path_publisher = self.create_publisher(
            Path,
            '/path',
            10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.flag = True

        self._action_client = ActionClient(self, FollowPath, 'follow_path')

        self._send_goal()   # Now working
        self.get_logger().info("Path!")

    def _send_goal(self):
        # Define the path
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        
        radius = (0.5 + 0.35) / 2
        step = 10
        left_turn = True

        tf_odom_base_link = None
        # rclpy.spin_once(self)

        while tf_odom_base_link is None:
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
                odom_base_link_msg = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                tf_odom_base_link = rnp.numpify(odom_base_link_msg.transform)
            except Exception as e:
                self.get_logger().info(f"{e}")
                rclpy.spin_once(self, timeout_sec=0.1)

        for angle in range(-90, 90, step):
            angle_rad = math.radians(angle)
            x = radius * math.cos(angle_rad)
            y = radius * math.sin(angle_rad)
            # self.get_logger().info("HERE")

            # waypoint_msg.pose.position.x = x
            # if left_turn:
            #     waypoint_msg.pose.position.y = y + radius
            # else:
            #     waypoint_msg.pose.position.y = y - radius

            # waypoint_msg.pose.orientation.w = 1.0

            # path_msg.poses.append(waypoint_msg)
            yaw = angle_rad + math.pi / 2
            # cos_yaw = math.cos(yaw)
            # sin_yaw = math.sin(yaw)

            # tf_base_link_point = np.eye(4)
            # tf_base_link_point[0, 0] = cos_yaw
            # tf_base_link_point[0, 1] = -sin_yaw
            # tf_base_link_point[1, 0] = sin_yaw
            # tf_base_link_point[1, 1] = cos_yaw

            rotation_matrix = euler2mat(0, 0, yaw, 'sxyz')
            tf_base_link_point = np.eye(4)
            tf_base_link_point[:3, :3] = rotation_matrix

            if left_turn:
                tf_base_link_point[:3, 3] = np.array([x , y + radius, 0])
            else:
                tf_base_link_point[:3, 3] = np.array([x , y - radius, 0])

            waypoint = tf_odom_base_link @ tf_base_link_point

            waypoint_msg_pose = rnp.msgify(Pose, waypoint)

            waypoint_msg_pose_stamped = PoseStamped()
            waypoint_msg_pose_stamped.header.frame_id = 'odom'
            waypoint_msg_pose_stamped.pose = waypoint_msg_pose

            path_msg.poses.append(waypoint_msg_pose_stamped)

        self.path_publisher.publish(path_msg)

        # Create the goal message
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg

        # Wait until the action server is ready and send the goal
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # while self.flag is True:
        #     rclpy.spin_once(self)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        # self.flag = False

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation Result: {result.result}')
        self.flag = False

def main(args=None):
    rclpy.init(args=args)
    node = FollowPathClient()
    while rclpy.ok() and node.flag:
        rclpy.spin(node)
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
