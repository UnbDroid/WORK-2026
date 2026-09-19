#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")

        self.arm_coord_pub = self.create_publisher(PointStamped, "arm_coordinates", 10)
        self.target_position = 'initial'

        self.targets = {
            'initial':   [0.0, -0.3, -0.35],
            'get_cube':  [0.0, 0.0, 0.27],
            'slot1':     [-0.10, -0.20, -0.30],
            'slot2':     [0.0, -0.20, -0.30],
            'slot3':     [0.10, -0.20, -0.30],
            'shelf':     [0.0, 0.5, 0.2],
            'drop_cube': [0.0, 0.0, 0.27],
        }

        # Publica periodicamente no tópico
        self.timer = self.create_timer(0.5, self.publish_target_coordinates)

    def publish_target_coordinates(self):
        coords = self.targets.get(self.target_position, [0.0, 0.0, 0.0])

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "arm_base"
        msg.point.x = float(coords[0])
        msg.point.y = float(coords[1])
        msg.point.z = float(coords[2])

        self.arm_coord_pub.publish(msg)

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
                    self.target_position = choice
                    self.get_logger().info(f"Target atualizado para: {choice} -> {self.targets[choice]}")
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