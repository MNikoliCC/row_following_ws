#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

import math

import time

from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory
import os

import pymap3d as pm
from nav2_test import FollowPathClient

package_dir = get_package_share_directory("row_following_bringup")
yolo_model_path = os.path.join(package_dir, 'resource', 'best.pt')

classNames = ['Ground', 'CilantroOm']

last_time = 0
integral = 0
previous = 0

kp = 0.5
ki = 0
kd = 0.1

def pid(error):
    global last_time, integral, previous
    now = time.time()
    dt = now - last_time / 1000.0
    last_time = now

    proportional = error
    integral += error * dt
    derivative = (error - previous) / dt
    previous = error

    output = kp * proportional + ki * integral + kd * derivative

    return output

class RowFollow(Node):
    def __init__(self):
        super().__init__('row_follow_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.image_subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            1)
        self.subscription = self.create_subscription(
            NavSatFix,
            '/navsatfix',
            self.navsat_callback,
            10
        )

        self.model = YOLO(yolo_model_path)
        self.mode = None
        self.out = False

        self.ref_lat = 45.2586161
        self.ref_lon = 19.8066591
        self.ref_alt = 76.0

        self.get_logger().info('Row Following Started!')

    def image_callback(self, msg):
        self.bridge = CvBridge()
        cmd_vel = Twist()
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, channels = rgb_image.shape
        cropped_rgb_image = rgb_image[int((height / 2) - 40) : int((height / 2) + 40), :] #Crop the image
        height, width, channels = cropped_rgb_image.shape
        center_coordinates = (width // 2, height // 2)

        # image = cv2.circle(cropped_rgb_image, center_coordinates, 5, (0, 0, 255), -1)

        results = self.model(cropped_rgb_image)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                w, h = x2 - x1, y2 - y1

                conf = math.ceil((box.conf[0] * 100)) / 100

                cls = int(box.cls[0])

                class_colors = {
                    0: (255, 0, 0),   # Blue for 'Ground'
                    1: (0, 255, 0)    # Green for 'CilantroOm'
                }
                color = class_colors.get(cls, (0, 0, 255))  # Default to red if class not found
                if cls == 0:
                    ground_center_coordinates = (x1 + (w // 2), y1 + (h // 2))
                    cv2.circle(cropped_rgb_image, ground_center_coordinates, 5, (255, 0, 0), -1)

                cv2.rectangle(cropped_rgb_image, (x1, y1), (x1 + w, y1 + h), color, 2)
                cv2.putText(cropped_rgb_image, f'{classNames[cls]} {conf}', (max(0, x1), max(35, y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.circle(cropped_rgb_image, center_coordinates, 5, (0, 0, 255), -1)

        cv2.line(cropped_rgb_image, (center_coordinates[0], height), center_coordinates, (0, 0, 255), 3)

        cv2.line(cropped_rgb_image, (center_coordinates[0], height), ground_center_coordinates, (255, 0, 0), 3)

        # adjacent = height - center_coordinates[1]
        adjacent = height / 2
        hypotenuse = math.sqrt((height / 2) ** 2 + (abs(center_coordinates[0] - ground_center_coordinates[0])) ** 2)

        angle_radians = math.acos(adjacent / hypotenuse)

        angle_degrees = math.degrees(angle_radians)

        # print(angle_degrees)

        cv2.putText(cropped_rgb_image, f'{angle_degrees:.2f}', (width // 2, max(35, y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.imshow("YOLO Output", cropped_rgb_image)
        cv2.waitKey(1)

        if self.mode == 'turn':
            return

        desired_value = 0.0
        actual_value = angle_radians
        error = abs(desired_value - actual_value)
        output = pid(error)

        cmd_vel.linear.x = 0.5

        if center_coordinates[0] > ground_center_coordinates[0]:
            cmd_vel.angular.z = output
            print(output)
        elif center_coordinates[0] < ground_center_coordinates[0]:
            cmd_vel.angular.z = -output
            print(-output)
        else:
            cmd_vel.angular.z = 0.0
            print("0.0")

        self.publisher_.publish(cmd_vel)

    def navsat_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        x, y, z = pm.geodetic2enu(lat=lat, lon=lon, h=alt,lat0=self.ref_lat, lon0=self.ref_lon, h0=self.ref_alt)
        if x >= 3.5 and self.out == False:
            self.out = True
            self.mode = 'turn'
            obj = FollowPathClient()
            while rclpy.ok() and obj.flag:
                rclpy.spin_once(obj, timeout_sec=0.1)
            self.get_logger().info('Path goal reached or aborted.')
            self.get_logger().info('End of the row!')
            self.mode = 'ml'
        else:
            self.mode = 'ml'

        # for transform in msg.transforms:
        #     # Check if the transform is from 'World' to 'base_link'
        #     if transform.header.frame_id == 'map' and transform.child_frame_id == 'base_link':
        #         x = transform.transform.translation.x
        #         if x >= 3.7:
        #             self.mode = False
        #             self.get_logger().info('End of the row!')
        #         else:
        #             self.mode = True
                

def main(args=None):
    rclpy.init(args=args)
    row_follow_node = RowFollow()
    while rclpy.ok():
        rclpy.spin(row_follow_node)
        # rclpy.spin_once(row_follow_node)
    row_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
