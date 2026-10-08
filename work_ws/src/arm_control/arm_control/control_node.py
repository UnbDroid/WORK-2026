#!/usr/bin/env python3

import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String

class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")

        # Publicadores para o hardware
        self.arm_coord_pub = self.create_publisher(Point, "arm_coordinates", 10)
        self.gripper_pub = self.create_publisher(Bool, "gripper_command", 10)
        
        # Comunicação com o PlanSys2 (C++)
        self.status_pub = self.create_publisher(Bool, "/arm_status", 10)
        self.create_subscription(String, "/arm_command", self.command_callback, 10)

        self.sequence_running = False
        
        self.targets = {
            'initial': [-0.291334, 0.0, 0.2075],
            'pre_initial': [-0.291334, 0.0, 0.32207],
            'zero_position': [0.378666, 0.00, 0.2075],
            'get_cube':  [0.3324, 0.00, 0.0497],
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
        
        self.get_logger().info("Control Node do Braço aguardando comandos do PlanSys2...")

    def publish_target_coordinates(self, target_name):
        coords = self.targets[target_name]
        msg = Point()
        msg.x, msg.y, msg.z = float(coords[0]), float(coords[1]), float(coords[2])
        self.arm_coord_pub.publish(msg)
        self.get_logger().info(f"Target: {target_name} -> x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}")

    def publish_gripper(self, open_gripper: bool):
        msg = Bool()
        msg.data = open_gripper 
        self.gripper_pub.publish(msg)

    def command_callback(self, msg):
        if self.sequence_running:
            self.get_logger().warn("Aviso: Braço já está em movimento. Comando ignorado.")
            return
            
        command = msg.data
        self.sequence_running = True
        
        # Dispara a coreografia em segundo plano
        threading.Thread(target=self.execute_sequence, args=(command,), daemon=True).start()

    def execute_sequence(self, command):
        self.get_logger().info(f"Iniciando coreografia: {command}")
        
        if command == "get_cube_table":
            self.publish_gripper(True) # Abre garra
            time.sleep(1.0)
            self.publish_target_coordinates('shelf') # Fica acima do cubo
            time.sleep(4.0)
            self.publish_target_coordinates('get_cube') # Desce na mesa
            time.sleep(4.0)
            self.publish_gripper(False) # Fecha garra
            time.sleep(1.0)
            self.publish_target_coordinates('pre_initial') # Recolhe
            time.sleep(4.0)
            self.publish_target_coordinates('initial') # Volta para posição inicial
            time.sleep(7.0)

        elif command.startswith("put_slot"):
            # O comando será algo como "put_slot1", "put_slot2"...
            slot_name = command.split("_")[1] # Extrai 'slot1'
            pre_slot_name = "pre_" + slot_name # Monta 'pre_slot1'
            
            self.publish_target_coordinates(pre_slot_name) # Fica acima do buraco
            time.sleep(4.0)
            self.publish_target_coordinates(slot_name) # Desce no buraco
            time.sleep(2.0)
            self.publish_gripper(True) # Solta
            time.sleep(1.0)
            self.publish_target_coordinates('pre_initial') # Recolhe
            time.sleep(4.0)
            self.publish_target_coordinates('initial') # Volta para posição inicial
            time.sleep(7.0)

        else:
            self.get_logger().error(f"Comando desconhecido: {command}")

        # Avisa o PlanSys2 (C++) que terminou!
        status_msg = Bool()
        status_msg.data = True
        self.status_pub.publish(status_msg)
        self.sequence_running = False
        self.get_logger().info(f"Coreografia {command} finalizada com sucesso.")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
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