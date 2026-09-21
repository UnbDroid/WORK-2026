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
            # 1. Em pé a 90° (x = L_offset, y = 0, z = H + L)
            'initial':   [0.05,  0.00,  0.40],

            # z, x, -y     
            # 2. Pegar o cubo no chão/esteira à frente
            'get_cube':  [0.26,  0.00,  0.03],

            # 3. Soltar nos 3 slots da plataforma de transporte
            'slot1':     [-0.25,  0.10,  0.12],
            'slot2':     [-0.25,  0.00,  0.12],
            'slot3':     [-0.25, -0.10,  0.12],

            # 4. Prateleira (shelf) alta à frente
            'shelf':     [0.27,  0.00,  0.27],

            # 5. Descarte
            'drop_cube': [0.26,  0.00,  0.03],
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