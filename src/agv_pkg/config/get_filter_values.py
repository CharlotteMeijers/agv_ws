#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import tf_transformations

class BoxMarkerPublisher(Node):
    def __init__(self):
        super().__init__('box_marker_publisher')

        # filter waardes
        self.min_x = -0.3
        self.max_x =  0.3
        self.min_y = -0.2
        self.max_y =  0.2
        self.min_z =  0
        self.max_z =  0.20
        self.frame_id = 'base_footprint'

        self.publisher = self.create_publisher(Marker, 'lidar_filter_box_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_box)

    def publish_box(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lidar_box"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = (self.min_x + self.max_x) / 2.0
        marker.pose.position.y = (self.min_y + self.max_y) / 2.0
        marker.pose.position.z = (self.min_z + self.max_z) / 2.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = abs(self.max_x - self.min_x)
        marker.scale.y = abs(self.max_y - self.min_y)
        marker.scale.z = abs(self.max_z - self.min_z)

        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)  
        marker.lifetime.sec = 0  

        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = BoxMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
