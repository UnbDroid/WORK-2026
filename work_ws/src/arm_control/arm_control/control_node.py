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

        self.arm_coord_pub = self.create_publisher(Point, "arm_coordinates", 10)
        self.gripper_pub = self.create_publisher(Bool, "gripper_command", 10)
        self.status_pub = self.create_publisher(Bool, "/arm_status", 10)
        self.create_subscription(String, "/arm_command", self.command_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.drop_counter = 0
        self.sequence_running = False

        self.targets = {
            'initial': [-0.291334, 0.0, 0.2075],
            'pre_initial': [-0.291334, 0.0, 0.32207],
            'zero': [0.378666, 0.00, 0.2075],
            'get_cube_15':  [0.348748, 0.0, 0.085718],
            'get_cube_10':  [0.3324, 0.0, 0.0697],
            'get_cube_5': [0.307641, 0.0, 0.021291],
            'teste_cube':  [0.3324, 0.00, 0.0497],
            'pre_slot1': [-0.370988,  0.060215, 0.32207],
            'pre_slot2': [-0.373852, -0.042785, 0.32207],
            'pre_slot3': [-0.356977, -0.143785, 0.32207],
            'put_slot1': [-0.370988, 0.060215, 0.164674],
            'put_slot2': [-0.373852, -0.042785, 0.164674],
            'put_slot3': [-0.356977, -0.143785, 0.16074],
            'get_slot1': [-0.370988, 0.060215, 0.1527],
            'get_slot2': [-0.373852, -0.042785, 0.1527],
            'get_slot3': [-0.356977, -0.143785, 0.15274],
            'pre_drop_cube_1': [0.3324,  0.15, 0.3228],
            'pre_drop_cube_2': [0.3324,  0.00, 0.3228],
            'pre_drop_cube_3': [0.3324, -0.15, 0.3228],
            'drop_cube_15_1': [0.398370, 0.15, 0.100718],
            'drop_cube_15_2': [0.398370, 0.0, 0.082018],
            'drop_cube_15_3': [0.398370, -0.15, 0.067718],
            'drop_cube_10_1': [0.3324, 0.15, 0.0747],
            'drop_cube_10_2': [0.3324, 0.0, 0.0777],
            'drop_cube_10_3': [0.3324, -0.15, 0.0617],
            'drop_cube_5_1': [0.307641, 0.15, 0.021291],
            'drop_cube_5_2': [0.307641, 0.0, 0.031291],
            'drop_cube_5_3': [0.307641, -0.15, 0.021291],
            'pre_drop_cube_shelf_1': [0.3003,  0.15, 0.5228],
            'pre_drop_cube_shelf_2': [0.3003,  0.00, 0.5228],
            'pre_drop_cube_shelf_3': [0.3003, -0.15, 0.5228],
            'drop_cube_shelf_1': [0.3003, 0.15, 0.4228],
            'drop_cube_shelf_2': [0.3003, 0.00, 0.4228],
            'drop_cube_shelf_3': [0.3003, -0.15, 0.4228],
            'pre_drop_container': [0.3324, 0.00, 0.3228],
            'drop_container': [0.3324, 0.00, 0.1027],
            'pre_stack_15': [0.398370, 0.00, 0.3228],
            'pre_stack_10': [0.3324, 0.00, 0.3228],
            'pre_stack_5':  [0.307641, 0.00, 0.3228],
            'stack_base_15': [0.398370, 0.00, 0.082018],
            'stack_base_10': [0.3324, 0.00, 0.0777],
            'stack_base_5':  [0.307641, 0.00, 0.031291],
            'stack_top_15': [0.398370, 0.00, 0.124018], 
            'stack_top_10': [0.3324, 0.00, 0.1197],     
            'stack_top_5':  [0.307641, 0.00, 0.073291], 
            'shelf': [0.3003, 0.00, 0.4228]
        }
        
        self.height_map = {'w1': '10', 'w2': '5', 'w3': '15', 'w4': '10', 'w5': '5', 'w6': '15'}
        self.get_logger().info("Control Node do Braço aguardando comandos do PlanSys2...")

    def publish_target_coordinates(self, target_name):
        coords = self.targets[target_name]
        msg = Point()
        msg.x, msg.y, msg.z = float(coords[0]), float(coords[1]), float(coords[2])
        self.arm_coord_pub.publish(msg)

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
        
        if command.startswith("get_cube_table"):
            workspace = command.split("_")[-1]
            h = self.height_map.get(workspace, '10')
            target_name = f'get_cube_{h}'
            
            self.get_logger().info(f"Pegando cubo na mesa {workspace} (Altura: {h}cm)")
            self.publish_gripper(True) 
            time.sleep(4.0)
            self.publish_target_coordinates(target_name)
            time.sleep(4.0)
            self.publish_gripper(False) 
            time.sleep(3.0)
            self.publish_target_coordinates('shelf')
            time.sleep(4.0)
            self.publish_target_coordinates('pre_initial') 
            time.sleep(4.0)

        elif command.startswith("put_slot"):
            base_slot = command.split("put_")[1] # Extrai 'slot1'
            pre_slot_name = "pre_" + base_slot # Monta 'pre_slot1'
            
            self.publish_target_coordinates(pre_slot_name) 
            time.sleep(4.0)
            self.publish_target_coordinates(command)
            time.sleep(4.0)
            self.publish_gripper(True) 
            time.sleep(3.0)
            self.publish_target_coordinates(pre_slot_name) 
            time.sleep(3.0)
            self.publish_target_coordinates('pre_initial') 
            time.sleep(4.0)
            self.publish_target_coordinates('zero') 
            time.sleep(2.0)

            self.get_logger().info("Dando ré de 20cm...")
            cmd = Twist()
            cmd.linear.x = -0.10
            start_time = time.time()
            duration = 2.0  # 1.0s a 0.20 m/s = 0.20m (20cm)
            
            while (time.time() - start_time) < duration:
                self.cmd_vel_pub.publish(cmd)
                time.sleep(0.1) 
            cmd.linear.x = 0.0
            self.cmd_vel_pub.publish(cmd)

        elif command.startswith("put_cube_table"):
            workspace = command.split("_")[-1] # Extrai 'w1', 'w2' ou 'w3'
            h = self.height_map.get(workspace, '10') # Busca se é 5, 10 ou 15cm

            self.drop_counter += 1
            if self.drop_counter > 3:
                self.drop_counter = 1

            target_name = f'drop_cube_{h}_{self.drop_counter}'
            pre_target_name = f'pre_drop_cube_{self.drop_counter}'
            
            self.publish_target_coordinates('shelf') 
            time.sleep(4.0)
            self.publish_target_coordinates(target_name)
            time.sleep(4.0)
            self.publish_gripper(True)
            time.sleep(3.0)
            self.publish_target_coordinates(pre_target_name)
            time.sleep(4.0)
            self.publish_target_coordinates('pre_initial')
            time.sleep(3.0)
            self.publish_target_coordinates('initial')
            time.sleep(2.0)

        elif command.startswith("put_cube_shelf"):
            self.drop_counter += 1
            if self.drop_counter > 3:
                self.drop_counter = 1

            target_name = f'drop_cube_shelf_{self.drop_counter}'
            pre_target_name = f'pre_drop_cube_shelf_{self.drop_counter}'
            
            self.publish_target_coordinates('shelf') 
            time.sleep(4.0)
            self.publish_target_coordinates(target_name)
            time.sleep(4.0)
            self.publish_gripper(True)
            time.sleep(4.0)
            self.publish_target_coordinates(pre_target_name)
            time.sleep(4.0)
            self.publish_target_coordinates('pre_initial')
            time.sleep(3.0)
            self.publish_target_coordinates('initial')
            time.sleep(4.0)

        elif command.startswith("put_cube_container"):

            parts = command.split("_")
            slot = parts[3]
            pre_slot_name = f'pre_{slot}'
            get_slot_name = f'get_{slot}'


            self.get_logger().info("Pegando cubo no slot")

            self.publish_gripper(True)
            time.sleep(2.0)
            self.publish_target_coordinates('pre_initial')
            time.sleep(4.0)
            self.publish_target_coordinates(pre_slot_name)
            time.sleep(4.0)
            self.publish_target_coordinates(get_slot_name)
            time.sleep(4.0)
            self.publish_gripper(False)
            time.sleep(4.0)
            self.publish_target_coordinates(pre_slot_name)
            time.sleep(4.0)
            self.publish_target_coordinates('pre_initial')
            time.sleep(3.0)

            self.get_logger().info("Soltando cubo no contêiner")

            self.publish_target_coordinates('pre_drop_container')
            time.sleep(4.0)
            self.publish_target_coordinates('drop_container')
            time.sleep(4.0)
            self.publish_gripper(True)
            time.sleep(4.0)
            self.publish_target_coordinates('pre_drop_container')
            time.sleep(3.0)
            self.publish_target_coordinates('pre_initial')
            time.sleep(3.0)
            self.publish_target_coordinates('initial')
            time.sleep(4.0)

            self.get_logger().info("Dando ré de 10cm...")
            cmd = Twist()
            cmd.linear.x = -0.10
            start_time = time.time()
            duration = 2.0  # 1.0s a 0.20 m/s = 0.20m (20cm)

            while (time.time() - start_time) < duration:
                self.cmd_vel_pub.publish(cmd)
                time.sleep(0.1) 
            cmd.linear.x = 0.0
            self.cmd_vel_pub.publish(cmd)

        elif command.startswith("get_slot"):
            base_slot = command.split("get_")[1]
            pre_slot_name = "pre_" + base_slot
            
            self.publish_gripper(True) 
            time.sleep(2.0)
            self.publish_target_coordinates(pre_slot_name)
            time.sleep(4.0)
            self.publish_target_coordinates(command)
            time.sleep(4.0)
            self.publish_gripper(False) 
            time.sleep(4.0)
            self.publish_target_coordinates(pre_slot_name) 
            time.sleep(4.0)
            self.publish_target_coordinates('pre_initial')
            time.sleep(3.0)

        # pega do slot e já coloca na mesa como base
        elif command.startswith("PLACE_BASE"):
            # Ex: "PLACE_BASE_w1_slot2"
            parts = command.split("_")
            workspace = parts[2]
            slot = parts[3] 
            h = self.height_map.get(workspace, '10') 
            
            target_name = f'stack_base_{h}'
            pre_target_name = f'pre_stack_{h}'
            pre_slot_name = f'pre_{slot}'
            get_slot_name = f'get_{slot}'
            
            self.publish_gripper(True) 
            time.sleep(2.0)
            self.publish_target_coordinates(pre_slot_name)
            time.sleep(4.0)
            self.publish_target_coordinates(get_slot_name)
            time.sleep(4.0)
            self.publish_gripper(False) 
            time.sleep(4.0)
            self.publish_target_coordinates(pre_slot_name) 
            time.sleep(4.0)
            
            self.publish_target_coordinates('shelf') 
            time.sleep(4.0)
            self.publish_target_coordinates(target_name)
            time.sleep(4.0)
            self.publish_gripper(True)
            time.sleep(4.0)
            self.publish_target_coordinates(pre_target_name)
            time.sleep(4.0)
            self.publish_target_coordinates('pre_initial')
            time.sleep(3.0)
            self.publish_target_coordinates('initial')
            time.sleep(4.0)

        # pega do slot e já empilha
        elif command.startswith("PLACE_TOP_OFFSET"):
            # Ex: "PLACE_TOP_OFFSET_w1_slot1"
            parts = command.split("_")
            workspace = parts[3] 
            slot = parts[4]
            h = self.height_map.get(workspace, '10') 
            
            target_name = f'stack_top_{h}'
            pre_target_name = f'pre_stack_{h}'
            pre_slot_name = f'pre_{slot}'
            get_slot_name = f'get_{slot}'
            
            self.publish_gripper(True) 
            time.sleep(2.0)
            self.publish_target_coordinates(pre_slot_name)
            time.sleep(4.0)
            self.publish_target_coordinates(get_slot_name)
            time.sleep(4.0)
            self.publish_gripper(False) 
            time.sleep(4.0)
            self.publish_target_coordinates(pre_slot_name) 
            time.sleep(4.0)
            
            self.publish_target_coordinates('shelf') 
            time.sleep(4.0)
            self.publish_target_coordinates(target_name)
            time.sleep(4.0)
            self.publish_gripper(True)
            time.sleep(4.0)
            self.publish_target_coordinates(pre_target_name)
            time.sleep(4.0)
            self.publish_target_coordinates('pre_initial')
            time.sleep(3.0)
            self.publish_target_coordinates('initial')
            time.sleep(4.0)

        elif command == "reset_arm":
            #time.sleep(20.0)
            self.publish_target_coordinates('pre_initial')
            time.sleep(3.0)
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
            return 

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