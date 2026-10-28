#!/usr/bin/env python3

import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")

        # Publicadores para o hardware
        self.arm_coord_pub = self.create_publisher(Point, "arm_coordinates", 10)
        self.gripper_pub = self.create_publisher(Bool, "gripper_command", 10)
        
        # Comunicação com o PlanSys2 (C++)
        self.status_pub = self.create_publisher(Bool, "/arm_status", 10)
        self.create_subscription(String, "/arm_command", self.command_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.drop_counter = 0

        self.sequence_running = False

        self.targets = {
            'initial': [-0.291334, 0.0, 0.2075],
            'pre_initial': [-0.291334, 0.0, 0.32207],
            'zero': [0.378666, 0.00, 0.2075],
            'get_cube_15':  [0.348748, 0.0, 0.120718],
            'get_cube_10':  [0.3324, 0.0, 0.0197],
            'get_cube_5': [0.307641, 0.0, 0.001291],
            'get_cube':  [0.3324, 0.00, 0.0097],
            'teste_cube':  [0.3324, 0.00, 0.0497],
            'pre_slot1': [-0.3530,  0.062215, 0.32207],
            'pre_slot2': [-0.3570, -0.037892, 0.32207],
            'pre_slot3': [-0.3350, -0.131892, 0.32207],

            'put_slot1': [-0.370988, 0.062215, 0.164674],
            'put_slot2': [-0.373852, -0.042785, 0.168674],
            'put_slot3': [-0.356977, -0.136785, 0.16574],

            'get_slot1': [-0.370988, 0.060215, 0.1347],
            'get_slot2': [-0.373852, -0.042785, 0.137674],
            'get_slot3': [-0.356977, -0.158785, 0.13474],

            'pre_drop_cube_1': [0.3324,  0.15, 0.4228],
            'pre_drop_cube_2': [0.3324,  0.00, 0.4228],
            'pre_drop_cube_3': [0.3324, -0.15, 0.4228],

            'drop_cube_15_1': [0.358370, 0.15, 0.120718],
            'drop_cube_15_2': [0.358370, 0.0, 0.120718],
            'drop_cube_15_3': [0.358370, -0.15, 0.120718],

            'drop_cube_10_1': [0.3324, 0.15, 0.0647],
            'drop_cube_10_2': [0.3324, 0.0, 0.0777],
            'drop_cube_10_3': [0.3324, -0.15, 0.0647],

            'drop_cube_5_1': [0.318082, 0.15, 0.015352],
            'drop_cube_5_2': [0.318082, 0.0, 0.015352],
            'drop_cube_5_3': [0.318082, -0.15, 0.015352],

            'shelf': [0.3003, 0.00, 0.4228]
        }
        
        # Mapeia qual mesa do PDDL representa qual altura no mundo real
        self.height_map = {
            'w1': '10',
            'w2': '10',
            'w3': '15'
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
        
        threading.Thread(target=self.execute_sequence, args=(command,), daemon=True).start()

    def execute_sequence(self, command):
        self.get_logger().info(f"Iniciando coreografia: {command}")
        
        # Agora é startswith porque a string chega como "get_cube_table_w1"
        if command.startswith("get_cube_table"):
            workspace = command.split("_")[-1] # Extrai 'w1'
            h = self.height_map.get(workspace, '10') # Pega a altura. Se não achar, usa 10cm por segurança
            target_name = f'get_cube_{h}'
            
            self.get_logger().info(f"Pegando cubo na mesa {workspace} (Altura: {h}cm)")
            self.publish_gripper(True) 
            time.sleep(1.0)
            self.publish_target_coordinates('shelf') 
            time.sleep(5.0)
            self.publish_target_coordinates(target_name)
            time.sleep(5.0)
            self.publish_gripper(False) 
            time.sleep(3.0)
            self.publish_target_coordinates('shelf')
            time.sleep(5.0)
            self.publish_target_coordinates('pre_initial') 
            time.sleep(5.0)

        elif command.startswith("put_slot"):
            base_slot = command.split("put_")[1] # Extrai 'slot1'
            pre_slot_name = "pre_" + base_slot # Monta 'pre_slot1'
            
            self.publish_target_coordinates(pre_slot_name) 
            time.sleep(5.0)
            self.publish_target_coordinates(command)
            time.sleep(4.0)
            self.publish_gripper(True) 
            time.sleep(3.0)
            self.publish_target_coordinates(pre_slot_name) 
            time.sleep(4.0)
            self.publish_target_coordinates('pre_initial') 
            time.sleep(4.0)
            self.publish_target_coordinates('initial') 
            time.sleep(5.0)
            self.publish_target_coordinates('zero') 
            time.sleep(5.0)

            # --- DAR RÉ PARA RECUPERAR A VISÃO ---
            self.get_logger().info("Dando ré de 20cm...")
            cmd = Twist()
            cmd.linear.x = -0.20
            
            # Publica o comando continuamente para evitar o corte de segurança do motor
            start_time = time.time()
            duration = 1.0  # 1.0s a 0.20 m/s = 0.20m (20cm)
            
            while (time.time() - start_time) < duration:
                self.cmd_vel_pub.publish(cmd)
                time.sleep(0.1)  # Frequência de 10Hz mantendo os motores vivos
                
            cmd.linear.x = 0.0
            self.cmd_vel_pub.publish(cmd)

        elif command.startswith("put_cube_table"):
            workspace = command.split("_")[-1] # Extrai 'w1', 'w2' ou 'w3'
            h = self.height_map.get(workspace, '10') # Busca se é 5, 10 ou 15cm

            self.drop_counter += 1
            if self.drop_counter > 3:
                self.get_logger().warn("Aviso: Número máximo de drops atingido. Comando ignorado.")
                self.sequence_running = False
                return

            target_name = f'drop_cube_{h}_{self.drop_counter}'
            self.get_logger().info(f"Soltando cubo na mesa {workspace} (Altura: {h}cm)")
                    
            self.publish_target_coordinates('shelf') 
            time.sleep(5.0)
            self.publish_target_coordinates(target_name) # Desce na altura certa
            time.sleep(4.0)
            self.publish_gripper(True) # Solta o cubo
            time.sleep(2.0)
            self.publish_target_coordinates('shelf') # Sobe para a altura de segurança
            time.sleep(4.0)
            self.publish_target_coordinates('pre_initial') # Recolhe
            time.sleep(4.0)
            self.publish_target_coordinates('initial') # Volta para posição inicial
            time.sleep(5.0)

        elif command.startswith("get_slot"):
            base_slot = command.split("get_")[1]
            pre_slot_name = "pre_" + base_slot
            
            self.publish_gripper(True) 
            time.sleep(1.0)
            self.publish_target_coordinates(pre_slot_name)
            time.sleep(4.0)
            self.publish_target_coordinates(command) # <--- USE O COMMAND AQUI! ('get_slot1')
            time.sleep(4.0)
            self.publish_gripper(False) 
            time.sleep(2.0)
            self.publish_target_coordinates(pre_slot_name) 
            time.sleep(4.0)
            self.publish_target_coordinates('pre_initial')
            time.sleep(4.0)
            self.publish_target_coordinates('zero') 
            time.sleep(4.0)

        elif command == "reset_arm":
            self.publish_target_coordinates('pre_initial')
            time.sleep(4.0)
            self.publish_target_coordinates('initial')
            time.sleep(3.0)
            self.publish_gripper(True) 
            time.sleep(1.0)

        elif command == "zero_position":
            time.sleep(1.0)
            self.publish_target_coordinates('pre_initial')
            time.sleep(4.0)
            self.publish_target_coordinates('zero')
            time.sleep(4.0)

        else:
            self.get_logger().error(f"Comando desconhecido: {command}")
            self.sequence_running = False
            return # Aborta se der erro

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