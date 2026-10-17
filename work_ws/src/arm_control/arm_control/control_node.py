#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")

        self.arm_coord_pub = self.create_publisher(Point, "arm_coordinates", 10)
        self.targets = {
            'initial': [-0.291334, 0.0, 0.2075],
            'zero_position': [0.378666, 0.0, 0.2075],
            'get_cube_15':  [0.348748, 0.0, ],
            'get_cube_10':  [0.3324, 0.0, 0.0497],
            'get_cube_5': [0.307641, 0.0, 0.001291],
            'drop_cube_10_r':  [0.3324, 0.15, 0.0497],
            'drop_cube_10_l':  [0.3324, -0.15, 0.0497],
            'pre_slot1': [-0.3530,  0.062108, 0.32207],
            'pre_slot2': [-0.3570, -0.032892, 0.32207],
            'pre_slot3': [-0.3350, -0.127892, 0.32207],
            'slot1': [-0.370988,  0.062215, 0.166674],
            'slot2': [-0.373852, -0.022785, 0.166674],
            'slot3': [-0.356977, -0.117785, 0.166674],
            'drop_cube_15': [0.358370, 0.0, 0.120718],
            'drop_cube_10': [0.3425, 0.0, 0.0604],
            'drop_cube_5': [0.318082, 0.0, 0.015352],
            'shelf': [0.3003, 0.00, 0.4228],
        }

    def publish_target_coordinates(self, target_name):
        coords = self.targets[target_name]

        msg = Point()

        msg.x = float(coords[0])
        msg.y = float(coords[1])
        msg.z = float(coords[2])

        self.arm_coord_pub.publish(msg)

        self.get_logger().info( f"Target atualizado: {target_name} -> " f"x={msg.x:.4f}, " f"y={msg.y:.4f}, " f"z={msg.z:.4f}" )

    def input_loop(self):
        """Roda em uma thread separada para não travar o loop de eventos do ROS."""
        print(f"\nPosições disponíveis: {list(self.targets.keys())}")
        while rclpy.ok():
            try:
                choice = input("\nDigite a pose desejada (ou 'q' para sair): ").strip()
                if choice == 'q':
                    rclpy.shutdown()
                    break
                if choice in self.targets:
                    self.publish_target_coordinates(choice)
                else:
                    print(f"Posição '{choice}' inválida!")
            except (EOFError, KeyboardInterrupt):
                break

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    # Dispara a leitura do terminal em segundo plano
    thread = threading.Thread(target=node.input_loop, daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
