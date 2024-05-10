#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_inverse, quaternion_multiply

class TransformPublisher(Node):
    def __init__(self):
        super().__init__('transform_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishing interval (0.1 seconds = 10 Hz)
        self.timer = self.create_timer(0.1, self.publish_transform)

    def publish_transform(self):
        try:
            # Lookup transform from map to base_link
            map_to_base_link = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())

            # Lookup transform from odom to base_link
            odom_to_base_link = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())

            # Invert the odom to base_link transform
            q_odom_to_base_link = [
                odom_to_base_link.transform.rotation.x,
                odom_to_base_link.transform.rotation.y,
                odom_to_base_link.transform.rotation.z,
                odom_to_base_link.transform.rotation.w
            ]
            inv_q_odom_to_base_link = quaternion_inverse(q_odom_to_base_link)

            # Create the inverse transform (base_link to odom)
            base_link_to_odom = TransformStamped()
            base_link_to_odom.header.frame_id = 'base_link'
            base_link_to_odom.child_frame_id = 'odom'
            base_link_to_odom.transform.translation.x = -odom_to_base_link.transform.translation.x
            base_link_to_odom.transform.translation.y = -odom_to_base_link.transform.translation.y
            base_link_to_odom.transform.translation.z = -odom_to_base_link.transform.translation.z
            base_link_to_odom.transform.rotation.x = inv_q_odom_to_base_link[0]
            base_link_to_odom.transform.rotation.y = inv_q_odom_to_base_link[1]
            base_link_to_odom.transform.rotation.z = inv_q_odom_to_base_link[2]
            base_link_to_odom.transform.rotation.w = inv_q_odom_to_base_link[3]

            # Combine map_to_base_link and base_link_to_odom using quaternion multiplication
            q_map_to_base_link = [
                map_to_base_link.transform.rotation.x,
                map_to_base_link.transform.rotation.y,
                map_to_base_link.transform.rotation.z,
                map_to_base_link.transform.rotation.w
            ]
            q_map_to_odom = quaternion_multiply(q_map_to_base_link, inv_q_odom_to_base_link)

            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = self.get_clock().now().to_msg()
            map_to_odom.header.frame_id = 'map'
            map_to_odom.child_frame_id = 'odom'
            map_to_odom.transform.translation.x = map_to_base_link.transform.translation.x + base_link_to_odom.transform.translation.x
            map_to_odom.transform.translation.y = map_to_base_link.transform.translation.y + base_link_to_odom.transform.translation.y
            map_to_odom.transform.translation.z = map_to_base_link.transform.translation.z + base_link_to_odom.transform.translation.z
            map_to_odom.transform.rotation.x = q_map_to_odom[0]
            map_to_odom.transform.rotation.y = q_map_to_odom[1]
            map_to_odom.transform.rotation.z = q_map_to_odom[2]
            map_to_odom.transform.rotation.w = q_map_to_odom[3]

            # Publish the computed map to odom transform
            self.tf_broadcaster.sendTransform(map_to_odom)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
