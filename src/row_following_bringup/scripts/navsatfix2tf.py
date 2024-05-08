#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix
import pymap3d as pm

class NavSatToTF(Node):
    def __init__(self):
        super().__init__('navsat_to_tf_node')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/navsatfix',
            self.navsat_callback,
            10
        )
        self.publisher = self.create_publisher(
            TFMessage,
            '/tf',
            10
        )

        self.ref_lat = 45.2586161
        self.ref_lon = 19.8066591
        self.ref_alt = 76.0

    def navsat_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        x, y, z = pm.geodetic2enu(lat=lat, lon=lon, h=alt,lat0=self.ref_lat, lon0=self.ref_lon, h0=self.ref_alt)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        tf_msg = TFMessage(transforms=[transform])
        self.publisher.publish(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    navsat_to_tf_node = NavSatToTF()
    rclpy.spin(navsat_to_tf_node)
    navsat_to_tf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
