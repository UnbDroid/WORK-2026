#!/usr/bin/env python3

import threading
import time
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")

        self.arm_coord_pub = self.create_publisher(Point, "arm_coordinates", 10)
        self.gripper_pub = self.create_publisher(Bool, "gripper_command", 10)
        self.aligned_sub = self.create_subscription(
            Bool, "/cube_aligned", self.aligned_callback, 10)

        self.sequence_running = False
        
        self.targets = {
            'initial': [-0.291334, 0.0, 0.2075],
            'zero_position': [0.378666, 0.00, 0.2075],
            'get_cube':  [0.3324, 0.00, 0.0497], # n muda muita coisa, deveria descer
            'teste_cube':  [0.3324, 0.00, 0.0497],
            'pre_slot1': [-0.3530,  0.062108, 0.32207],
            'pre_slot2': [-0.3570, -0.032892, 0.32207],
            'pre_slot3': [-0.3350, -0.127892, 0.32207],
            'slot1': [-0.370988,  0.062215, 0.166674],
            'slot2': [-0.374738, -0.032785, 0.166674],
            'slot3': [-0.353799, -0.127785, 0.166674],
            'drop_cube_10': [0.3324, 0.00, 0.0497],
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

    def publish_gripper(self, open_gripper: bool):
        msg = Bool()
        msg.data = open_gripper   # True = abre, False = fecha (como no firmware)
        self.gripper_pub.publish(msg)

    def aligned_callback(self, msg):
        if not msg.data or self.sequence_running:
            return
        self.sequence_running = True
        threading.Thread(target=self.pick_sequence, daemon=True).start()

    def pick_sequence(self):
        self.get_logger().info("Alinhado! Iniciando sequência.") # 
        self.publish_target_coordinates('initial')      # levanta / recolhe
        time.sleep(4.0)
        self.publish_gripper(True)                    # abre a garra
        time.sleep(1.0)
        self.publish_target_coordinates('pre_slot1')     # vai até o cubo
        time.sleep(4.0)
        
        self.get_logger().info("Sequência concluída.")
        self.sequence_running = False    

    def input_loop(self): # meio q não precisa
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
