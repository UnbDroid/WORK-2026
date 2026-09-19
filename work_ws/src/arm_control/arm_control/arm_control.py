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

        self.declare_parameter(
            'target_position', 
            'home',
            ParameterDescriptor(description='pose')
        )

        target = self.get_parameter('target_position').get_parameter_value().string_value
        self.get_logger().info(f'Posição alvo selecionada: {target}')

        self.timer = self.create_timer(1.0, self.publish_target_coordinates)

    def publish_target_coordinates(self):
        targets = {
            'initial': [0.0, -0.3, -0.35],
            'get_cube': [0.0, 0.0, 0.27],
            'slot1': [-0.10, -0.20, -0.30],
            'slot2': [0.0, -0.20, -0.30],
            'slot3': [0.10, -0.20, -0.30],
            'shelf': [0, 0.5, 0.2],
            'drop_cube': [0.0, 0.0, 0.27],
        }

        coordinates = targets.get(self.target_position, [0.0, 0.0, 0.0])

        coordinates_msg = PointStamped()
        coordinates_msg.header.stamp = self.get_clock().now().to_msg()
        coordinates_msg.header.frame_id = "arm_base"
        coordinates_msg.point.x = float(coordinates[0])
        coordinates_msg.point.y = float(coordinates[1])
        coordinates_msg.point.z = float(coordinates[2])

        self.arm_coord_pub.publish(coordinates_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()