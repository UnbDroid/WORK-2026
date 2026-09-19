#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
import apriltag
import numpy as np

from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import ParameterDescriptor

class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")

        self.arm_coord_pub = self.create_publisher(PointStamped, "arm_coordinates", 10)

        self.get_logger().info("Subscriber 'cube_coordinates' inicializado!")
        self.get_logger().info("Publisher 'arm_coordinates' inicializado!")

        self.declare_parameter(
            'target_position', 
            'home',
            ParameterDescriptor(description='pose')
        )

        target = self.get_parameter('target_position').get_parameter_value().string_value

        self.get_logger().info(f'Posição alvo selecionada: {target}')
        self.target_position = target

    def coordinates_callback(self, msg):
        if self.target_position == 'initial':
            coordinates = [0.0, 0.0, 0.0]

        if self.target_position == 'get_cube':
            coordinates = [0.1, 0.2, 0.3]

        if self.target_position == 'slot1': 
            coordinates = [0.4, 0.5, 0.6]

        if self.target_position == 'slot2':
            coordinates = [0.7, 0.8, 0.9]

        if self.target_position == 'slot3':
            coordinates = [1.0, 1.1, 1.2]

        if self.target_position == 'shelf':
            coordinates = [1.3, 1.4, 1.5]

        if self.target_position == 'drop_cube':
            coordinates = [1.6, 1.7, 1.8]

        coordinates_msg = PointStamped()
        coordinates_msg.header.stamp = self.get_clock().now().to_msg()
        coordinates_msg.point.x = coordinates[0]
        coordinates_msg.point.y = coordinates[1]
        coordinates_msg.point.z = coordinates[2]
        
        self.arm_coord_pub.publish(coordinates_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()