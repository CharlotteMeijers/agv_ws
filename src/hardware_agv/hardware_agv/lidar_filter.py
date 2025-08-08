#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

class LaserBoxFilterNode(Node):
    def __init__(self):
        super().__init__('laser_box_filter')

        #Box that should not be taken into account
        self.min_x = -0.32
        self.max_x =  0.32
        self.min_y = -0.22
        self.max_y =  0.22
        self.min_z =  0.0
        self.max_z =  0.2
        self.box_frame = 'base_footprint'

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(LaserScan, input_topic, self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, output_topic, 10)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("LaserBoxFilter active")

    def scan_callback(self, msg: LaserScan):
        try:
            #Transform from laser to box
            transform = self.tf_buffer.lookup_transform(
                self.box_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"TF box filter failed: {e}")
            return

        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = list(msg.ranges)
        filtered_scan.intensities = list(msg.intensities)

        angle = msg.angle_min
        for i in range(len(msg.ranges)):
            r = msg.ranges[i]
            if math.isfinite(r):
                #Point in laser frame
                x_laser = r * math.cos(angle)
                y_laser = r * math.sin(angle)
                point = geometry_msgs.msg.PointStamped()
                point.header = msg.header
                point.point.x = x_laser
                point.point.y = y_laser
                point.point.z = 0.0

                try:
                    #Transform the point to the box frame
                    point_transformed = tf2_geometry_msgs.do_transform_point(point, transform)
                    x = point_transformed.point.x
                    y = point_transformed.point.y
                    z = point_transformed.point.z

                    #Delete the point if it is in the box boundaries
                    if (self.min_x <= x <= self.max_x and
                        self.min_y <= y <= self.max_y and
                        self.min_z <= z <= self.max_z):
                        filtered_scan.ranges[i] = float('nan')
                except Exception as e:
                    self.get_logger().info(f"Point transformation error: {e}")
                    pass

            angle += msg.angle_increment

        self.pub.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    node = LaserBoxFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

