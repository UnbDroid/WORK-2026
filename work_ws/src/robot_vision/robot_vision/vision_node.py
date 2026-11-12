#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
import apriltag
import numpy as np

from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import Bool, String
from cube_msgs.msg import Cube
from cube_msgs.msg import CubeArray

class VisionNode(Node):

    def __init__(self):
        super().__init__("vision_node")

        self.tag_size = 0.06  

        options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(options)

        self.coord_pub = self.create_publisher(PointStamped, "cube_coordinates", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cube_data_pub = self.create_publisher(CubeArray, "cube_data", 10)

        # PONTE COM O ACTION_NODE
        self.vision_active = False 
        self.current_target_id = -1
        self.create_subscription(String, "/vision_target", self.target_callback, 10)
        self.status_pub = self.create_publisher(Bool, "/vision_status", 10)

        self.last_seen = None
        self.lost_timeout = 2.0

        self.mode = "pick"
        self.container_color = None
        self.profiles = {
            "pick":      dict(target_z=0.412, x_offset=0.10),
            "stack":     dict(target_z=0.412, x_offset=0.10),   # calibrar com a pilha
            "container": dict(target_z=0.30,  x_offset=0.0),    # calibrar com a caixa
        }

        self.search_cube_enabled = False

        self.container_detection_enabled = False
        self.container_color = "blue"
        #self.create_subscription(String, "container_target", self.container_target_callback, 10)
        #self.create_subscription(Bool, "search_cube_enabled", self.search_cube_enabled_callback, 10)
        self.container_coord_pub = self.create_publisher(PointStamped, 'container_coordinates', 10)


        #self.precision_table_enabled = False
        #self.create_subscription(Bool, "precision_table_enabled", self.precision_table_enabled_callback, 10)
        #self.precision_coord_pub = self.create.publisher(PointStamped, 'table_tag_coordinates', 10)

        self.tag_type = "Unknown"

        self.get_logger().info("Publishers 'cube_coordinates' e 'cube_data' inicializados!")

        self.cube_aligned = False
        
        # Estados possíveis: "IDLE", "RETURNING", "SEARCHING", "ALIGNING"
        self.state = "IDLE" 
        self.search_start_time = None
        self.time_spent_searching = 0.0
        self.return_start_time = None
        self.wait_time = 3.0
        self.wait_start = None
        self.last_vision_usage = 0.0

        self.max_speed = 0.07     # m/s

        self.indice_camera = 2 # Índice na raspberry pi (diferente no notebook)
        self.cap = cv2.VideoCapture(self.indice_camera, cv2.CAP_V4L2)

        # TODO: Ver se isso realmente é necessário
        self.fx, self.fy = 500.0, 500.0
        self.cx0, self.cy0 = 320.0, 240.0
        self.container_long_side = 0.160   # m (dimensão externa maior)
        self.hits = 0
        self.prev_c = None

        self.red_lower = np.array([136, 87, 111], np.uint8)
        self.red_upper = np.array([180, 255, 255], np.uint8)
        self.red_lower2 = np.array([170, self.red_lower[1], self.red_lower[2]], np.uint8)
        self.red_upper2 = np.array([179, self.red_upper[1], self.red_upper[2]], np.uint8)

        self.blue_lower = np.array([87, 81, 100], np.uint8)
        self.blue_upper = np.array([141, 255, 205], np.uint8)



        if not self.cap.isOpened():
            self.get_logger().error(f"Não foi possível abrir a câmera no índice {self.indice_camera}!")
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(0.033, self.process_frame_callback)

    '''def target_callback(self, msg):
        tag_str = msg.data.replace("tag", "") 
        try:
            self.current_target_id = int(tag_str)
            self.vision_active = True
            self.cube_aligned = False
            self.last_seen = self.get_clock().now().nanoseconds / 1e9

            tempo_ocioso = self.last_seen - self.last_vision_usage

            # Se já gastamos tempo buscando antes, precisamos voltar primeiro
            if self.time_spent_searching > 0.1 and tempo_ocioso < 80.0:
                
                self.state = "RETURNING"
                self.return_start_time = self.last_seen
                self.get_logger().info(f'Visão ACORDADA! Retornando por {self.time_spent_searching:.2f}s antes de buscar a Tag {self.current_target_id}')
                self.last_vision_usage = 0
            else:
                self.state = "SEARCHING"
                self.search_start_time = self.last_seen
                self.get_logger().info(f'Visão ACORDADA! Iniciando busca direta pela Tag {self.current_target_id}')
                
        except ValueError:
            self.get_logger().error(f"Falha ao extrair ID da tag enviada: {msg.data}")'''

    def target_callback(self, msg):
        data = msg.data
        mode, target = data.split(":", 1) if ":" in data else ("pick", data)

        if mode not in self.profiles:
            self.get_logger().error(f"Modo desconhecido: {data}")
            return

        self.mode = mode
        self.vision_active = True
        self.cube_aligned = False
        self.last_seen = self.get_clock().now().nanoseconds / 1e9

        if mode == "container":
            self.container_color = target.replace("container-", "")
            self.state = "SEARCHING"
            self.search_start_time = self.last_seen
            self.get_logger().info(f"Modo CONTAINER: cor {self.container_color}")
            return

        # pick / stack: mesma lógica de tag que você já tinha
        try:
            self.current_target_id = int(target.replace("tag", ""))
        except ValueError:
            self.get_logger().error(f"Falha ao extrair ID da tag: {data}")
            return

        tempo_ocioso = self.last_seen - self.last_vision_usage
        if self.time_spent_searching > 0.1 and tempo_ocioso < 80.0:
            self.state = "RETURNING"
            self.return_start_time = self.last_seen
            self.last_vision_usage = 0
        else:
            self.state = "SEARCHING"
            self.search_start_time = self.last_seen
        self.get_logger().info(f"Modo {mode.upper()}: tag {self.current_target_id}")


    def process_frame_callback(self):
        if not self.vision_active:
            self.cap.grab()
            return

        agora = self.get_clock().now().nanoseconds / 1e9
        cmd = Twist()

        # --- MÁQUINA DE ESTADOS: RETORNO À ORIGEM ---
        if self.state == "RETURNING":
            tempo_retornando = agora - self.return_start_time
            if tempo_retornando < self.time_spent_searching:
                cmd.linear.y = -0.07 # Volta para a esquerda (negativo)
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info(f"Retornando à base... {tempo_retornando:.1f}/{self.time_spent_searching:.1f}s", throttle_duration_sec=0.5)
                self.cap.grab() # Limpa o buffer
                return
            else:
                # Chegou na base! Entra no estado de ESPERA
                cmd.linear.y = 0.0
                self.cmd_vel_pub.publish(cmd)
                self.time_spent_searching = 0.0
                
                self.state = "WAITING"
                self.wait_start = agora
                self.get_logger().info(f"Retorno concluído. Aguardando {self.wait_time}s para estabilizar...")
                return
                
        # --- MÁQUINA DE ESTADOS: ESPERA (NOVO) ---
        elif self.state == "WAITING":
            if agora - self.wait_start < self.wait_time:
                cmd.linear.y = 0.0
                self.cmd_vel_pub.publish(cmd)
                self.cap.grab() # Limpa o buffer
                return
            else:
                # Fim da espera, inicia a nova busca
                self.state = "SEARCHING"
                self.search_start_time = agora
                self.get_logger().info(f"Espera concluída. Iniciando busca pela Tag {self.current_target_id}.")
                # Não damos return aqui para ele fluir direto para a leitura do frame logo abaixo

        # Daqui pra frente, estamos em SEARCHING ou ALIGNING
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.cap.grab()
            return

        display_frame = frame.copy()

        if self.mode == "container":
            self.handle_container(frame, display_frame, agora)
        else:
            self.handle_tag(frame, display_frame, agora)

        cv2.imshow("Deteccao de Cubos - UnbDroid", display_frame)
        cv2.waitKey(1)
        return

    def handle_tag(self, frame, display_frame, agora):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = [] if self.cube_aligned else self.detector.detect(gray)
        ids_vistos = [d.tag_id for d in detections]
        self.get_logger().info(f"Tags no frame: {ids_vistos} | alvo atual: {self.current_target_id}", throttle_duration_sec=0.5)
        
        coord_msg = PointStamped()
        coord_msg.header.stamp = self.get_clock().now().to_msg()
        coord_msg.header.frame_id = "webcam_link"
        
        array_msg = CubeArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = "webcam_link"

        alvo = next((det for det in detections if det.tag_id == self.current_target_id), None)

        cmd = Twist()  # Inicializa o comando de velocidade

        # Só ativar na mesa de procura
        if alvo is not None:
            if self.search_cube_enabled:
                self.detect_tag(alvo, agora, display_frame, coord_msg, array_msg)
            '''elif self.precision_table_enabled:
                self.get_logger().info("Entrou no caso da mesa de precisão")
                cubo = self.precision_table(alvo, display_frame)
                self.detect_tag(alvo, agora, display_frame, coord_msg, array_msg)'''
                

        elif self.state == "SEARCHING":
            # Não achou a tag ainda, continua indo para a direita
            cmd.linear.y = 0.07
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info("Buscando tag para a direita...", throttle_duration_sec=1.0)
            
        elif self.state == "ALIGNING":
            perdido = self.last_seen is None or (agora - self.last_seen) > self.lost_timeout
            if perdido:
                # Se perdeu durante o alinhamento, volta pra busca
                self.state = "SEARCHING"
                # Reativa o contador de tempo de busca a partir de onde parou
                self.search_start_time = agora - self.time_spent_searching
                self.get_logger().info("Alvo perdido durante alinhamento. Retomando busca para a direita...", throttle_duration_sec=1.0)

    def handle_container(self, frame, display_frame, agora):
        cmd = Twist()
        container = self.color_detection(display_frame, cor=self.container_color)

        if container is not None:
            x, y, w, h, cx, cy = container
            if self.prev_c is not None \
                    and abs(cx - self.prev_c[0]) < 25 and abs(cy - self.prev_c[1]) < 25:
                self.hits += 1
            else:
                self.hits = 1
            self.prev_c = (cx, cy)
        else:
            self.hits, self.prev_c = 0, None

        confirmado = container is not None and self.hits >= 3

        # --- alinhamento: só com detecção confirmada ---
        if confirmado:
            xyz = self.container_pixels_to_xyz(w, h, cx, cy)
            if xyz is None:
                return
            x_m, y_m, z_m = xyz

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'webcam_link'
            msg.point.x, msg.point.y, msg.point.z = float(x_m), float(y_m), float(z_m)
            self.container_coord_pub.publish(msg)

            if self.state == "SEARCHING":
                self.state = "ALIGNING"
                self.time_spent_searching += agora - self.search_start_time
            self.last_seen = agora

            self.alignment(-1, x_m, y_m, z_m, **self.profiles["container"])

        # --- viu algo, mas ainda não confirmou: para e espera ---
        elif container is not None and self.state == "SEARCHING":
            self.cmd_vel_pub.publish(Twist())

        # --- nada visto ---
        elif self.state == "SEARCHING":
            cmd.linear.y = 0.07
            self.cmd_vel_pub.publish(cmd)

        elif self.state == "ALIGNING" and (agora - self.last_seen) > self.lost_timeout:
            self.state = "SEARCHING"
            self.search_start_time = agora


    def container_pixels_to_xyz(self, w_px, h_px, cx_px, cy_px):
        lado_px = max(w_px, h_px)
        if lado_px < 1:
            return None

        z = self.fx * self.container_long_side / lado_px
        x = (cx_px - self.cx0) * z / self.fx
        y = (cy_px - self.cy0) * z / self.fy
        return x, y, z

    def detect_tag(self, alvo, agora, display_frame, coord_msg, array_msg):
        # Achou a tag! Muda pro estado de alinhamento
        if self.state == "SEARCHING":
            self.state = "ALIGNING"
            # Congela o tempo que passamos buscando para podermos voltar depois
            self.time_spent_searching = agora - self.search_start_time
            self.get_logger().info(f"Tag {self.current_target_id} encontrada após {self.time_spent_searching:.2f}s! Iniciando alinhamento fino.")

        self.last_seen = agora

        tag_id = alvo.tag_id
        camera_params = [500.0, 500.0, 320.0, 240.0] 
        pose, _, _ = self.detector.detection_pose(
            alvo, camera_params, tag_size=self.tag_size
        )

        x_m = pose[0][3] 
        y_m = pose[1][3] 
        z_m = pose[2][3] 
        '''R = pose[:3, :3]
        yaw = np.arctan2(R[1, 0], R[0, 0])'''

        coord_msg.point.x = float(x_m)
        coord_msg.point.y = float(y_m)
        coord_msg.point.z = float(z_m)
        self.coord_pub.publish(coord_msg)

        cube_msg = Cube()
        cube_msg.id = int(tag_id)
        cube_msg.waypoint = "default"  
        cube_msg.color = "cor"  
        array_msg.cubes.append(cube_msg)

        self.render_preview(display_frame, alvo, tag_id, x_m, y_m, z_m)
        self.alignment(tag_id, x_m, y_m, z_m, **self.profiles[self.mode])

    def precision_table(self, display_frame, x, y, w, h):
        roi = display_frame[y:y+h, x:x+w] # checar valores

        if self.color_detection(roi) is not None:
            self.get_logger().info("Cubo detectado!")
            self.get_logger().info("Alinhando com o cubo. Depois tem q pegar")
            
            return True
        else:
            self.get_logger().info("Cubo não detectado na mesa de precisão.")
            return False


    def color_detection(self, roi, cor="None"): # ROI menor quando for detectar cor de cubo (TODO: testar valores)
        hsv_frame = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv_frame, self.red_lower, self.red_upper),
            cv2.inRange(hsv_frame, self.red_lower2, self.red_upper2))
        
        blue_mask = cv2.inRange(hsv_frame, self.blue_lower, self.blue_upper)

        if cor == "red":
            mask = red_mask
        elif cor == "blue":
            mask = blue_mask
        else:
            return None
        
        if cv2.countNonZero(mask) < 1000:
            return None

        # Reduz ruído
        kernel_open = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        kernel_close = np.ones((9, 9), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filtra contornos pequenos
        contours = [c for c in contours if cv2.contourArea(c) > 1000]

        if len(contours) == 0:
            return None

        # Seleciona o maior contorno
        contour = max(contours, key=cv2.contourArea)

        x, y, w, h = cv2.boundingRect(contour)

        cx = x + w // 2
        cy = y + h // 2

        centro = (cx, cy)

        cv2.imshow("Mascara Container", mask)
        self.render_container_preview(roi, (x, y, w, h), cor, centro)

        return x, y, w, h, cx, cy


    def alignment(self, tag_id, x, y, z, target_z = 0.412, x_offset = 0.10):
        tol_x = 0.012
        tol_z = 0.03    
        #tol_yaw = 0.07

        k_x = 0.42
        k_z = 0.3 
        #k_yaw = 0.3       
        max_speed = self.max_speed   

        erro_x = -x + x_offset
        erro_z = z - target_z
        #erro_yaw = yaw

        cmd = Twist()

        aligned_x = abs(erro_x) <= tol_x
        aligned_z = abs(erro_z) <= tol_z
        #aligned_yaw = abs(erro_yaw) <= tol_yaw

        if aligned_x and aligned_z: 
            self.cube_aligned = True

            cmd.linear.x = 0.0
            cmd.linear.y = 0.0

            self.cmd_vel_pub.publish(cmd)

            self.get_logger().info(f"Tag ID {tag_id} ALINHADA! Robô parado.")
            
            status_msg = Bool()
            status_msg.data = True
            self.status_pub.publish(status_msg)
            
            self.vision_active = False 
            self.state = "IDLE"

            self.last_vision_usage = self.get_clock().now().nanoseconds / 1e9
            
            return True

        if not aligned_x:
            cmd.linear.y = float(np.clip(k_x * erro_x, -max_speed, max_speed))

        if not aligned_z:
            cmd.linear.x = float(np.clip(k_z * erro_z, -max_speed, max_speed))

        self.get_logger().info(f"Alinhando Tag {tag_id} -> CmdVel: vx={cmd.linear.x:.2f}, vy={cmd.linear.y:.2f}", throttle_duration_sec=0.2)
        self.cmd_vel_pub.publish(cmd)


    def render_preview(self, image, detection, tag_id, x, y, z):
        corners = detection.corners.astype(int)
        for i in range(4):
            p1 = tuple(corners[i])
            p2 = tuple(corners[(i + 1) % 4])
            cv2.line(image, p1, p2, (0, 255, 0), 2)

        center = (int(detection.center[0]), int(detection.center[1]))
        cv2.circle(image, center, 5, (0, 0, 255), -1)
        
        texto_id = f"ID: {tag_id}"
        texto_coords = f"X:{x:.1f} Y:{y:.1f} Z:{z:.1f} m"

        pos_id = (center[0] - 40, center[1] - 25)
        pos_coords = (center[0] - 75, center[1] - 10)

        cv2.putText(image, texto_id, pos_id, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(image, texto_coords, pos_coords, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
        cv2.putText(image, texto_id, pos_id, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(image, texto_coords, pos_coords, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    def render_container_preview(self, image, contorno, cor, centro):
        x, y, w, h = contorno
        cx, cy = centro
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(image, centro, 5, (0, 0, 255), -1)
        cv2.putText(image, f"Centro: ({cx}, {cy}) px", (x, max(20, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
        cv2.putText(image, f"Centro: ({cx}, {cy}) px", (x, max(20, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)



    def __del__(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()



def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_node.destroy_node()
        cv2.destroyAllWindows() 
        rclpy.shutdown()

if __name__ == "__main__":
    main()
