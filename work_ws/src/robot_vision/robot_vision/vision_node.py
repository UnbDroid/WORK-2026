#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
import apriltag
import numpy as np

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
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

        self.detection_led_pub = self.create_publisher(Bool, "/detection_led", 10)
        self.detection_led_state = False

        # PONTE COM O ACTION_NODE
        self.vision_active = False 
        self.current_target_id = -1
        self.current_target_color = None
        self.create_subscription(String, "/vision_target", self.target_callback, 10)
        self.status_pub = self.create_publisher(Bool, "/vision_status", 10)

        self.create_subscription(Bool, "/precision_status", self.precision_status_callback, 10)

        self.slot_tags = [101, 102, 103, 104, 105, 106, 107]
        self.n_posicoes = len(self.slot_tags)
        self.pos_vazia_final = self.n_posicoes - 1   # índice da 107
        self.end_pos = None                          # posição onde termina o ciclo atual
        self.slot_toggle = 1
        self.arm_result = None
        self.cycle_start = None
        self.last_id = None
        self.primeiro_do_ciclo = True
        

        self.last_seen = None
        self.lost_timeout = 2.0
        self.direction = 1

        self.mode = "pick"
        self.container_color = None
        self.profiles = {
            "pick":      dict(target_z=0.40, x_offset=0.10),
            "stack":     dict(target_z=0.403, x_offset=0.10),
            "container": dict(target_z=0.232,  x_offset=0.09),
            "precisao": dict(target_z=0.40, x_offset=0.10)

        }

        
        self.container_coord_pub = self.create_publisher(PointStamped, 'container_coordinates', 10)
        self.info_pub = self.create_publisher(String, "/precision_info", 10)
        self.current_fase = None
        self.current_slot = None

        self.realocable = []
        self.counter = 0
        self.tag_save = False

        self.get_logger().info("Publishers 'cube_coordinates' e 'cube_data' inicializados!")

        self.cube_aligned = False
        self.aligned_pub = self.create_publisher(Bool, "/cube_aligned", 10)
        
        
        # Estados possíveis: "IDLE", "RETURNING", "SEARCHING", "ALIGNING"
        self.state = "IDLE" 
        self.search_start_time = None
        self.time_spent_searching = 0.0
        self.return_start_time = None
        self.wait_time = 3.0
        self.wait_start = None
        self.last_vision_usage = 0.0

        # Odometria lateral estimada a partir dos cmd_vel publicados (m, + = direita).
        # Soma busca E alinhamento, então o retorno volta exatamente pro início.
        self.desloc_y = 0.0
        self.last_vy = 0.0
        self.last_cmd_t = None
        self.margem_retorno = 0.0   # m extra para a esquerda no retorno (ajuste se ainda faltar)
        self.tol_retorno = 0.005    # m

        self.max_speed = 0.07     # m/s

        # --- CORREÇÃO DE YAW PELO LIDAR (mantém o robô paralelo à borda da mesa) ---
        # Por padrão a posição/rotação do lidar vem do TF (URDF: base_link -> rplidar_a1m8_1),
        # o mesmo que o rf2o usa. Então NÃO precisa de offset nem de "invertido" na mão.
        # Só se o TF não existir ele usa lidar_offset_deg / lidar_invertido.
        self.declare_parameter("usar_lidar", True)
        self.declare_parameter("scan_topic", "/scan_filtered")  # já sem o corpo do robô
        self.declare_parameter("usar_tf", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lidar_offset_deg", 0.0)   # reserva, só sem TF
        self.declare_parameter("lidar_invertido", False)  # reserva, só sem TF
        self.declare_parameter("setor_deg", 20.0)         # usa os pontos entre -setor e +setor na frente
        self.declare_parameter("dist_min", 0.10)          # m (a partir do lidar), ignora o próprio robô
        self.declare_parameter("dist_max", 1.5)           # m, ignora pontos mais longe que isso
        self.declare_parameter("min_pontos", 15)
        self.declare_parameter("yaw_max_deg", 25.0)       # leitura acima disso é descartada (segurança)
        self.declare_parameter("k_yaw", 0.8)              # (rad/s) por rad de erro
        self.declare_parameter("max_wz", 0.3)             # rad/s
        self.declare_parameter("tol_yaw_deg", 2.0)        # tolerância para considerar alinhado
        self.declare_parameter("zona_morta_deg", 0.5)     # abaixo disso não corrige (evita tremedeira)
        self.declare_parameter("sinal_yaw", 1.0)          # troque para -1.0 se ele girar para o lado errado
        p = lambda n: self.get_parameter(n).value
        self.usar_lidar = bool(p("usar_lidar"))
        self.usar_tf = bool(p("usar_tf"))
        self.base_frame = str(p("base_frame"))
        self.lidar_offset = np.radians(float(p("lidar_offset_deg")))
        self.lidar_invertido = bool(p("lidar_invertido"))
        self.setor = np.radians(float(p("setor_deg")))
        self.dist_min = float(p("dist_min"))
        self.dist_max = float(p("dist_max"))
        self.min_pontos = int(p("min_pontos"))
        self.yaw_max = np.radians(float(p("yaw_max_deg")))
        self.k_yaw = float(p("k_yaw"))
        self.max_wz = float(p("max_wz"))
        self.tol_yaw = np.radians(float(p("tol_yaw_deg")))
        self.zona_morta = np.radians(float(p("zona_morta_deg")))
        self.sinal_yaw = float(p("sinal_yaw"))
        self.yaw_mesa = None        # rad, 0 = paralelo à mesa; None = sem leitura confiável
        self.yaw_stamp = 0.0
        self.tf_lidar = None        # (R 3x3, t 3) base_frame <- frame do lidar
        if self.usar_lidar:
            if self.usar_tf:
                from tf2_ros import Buffer, TransformListener
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(LaserScan, p("scan_topic"), self.scan_callback,
                                     qos_profile_sensor_data)

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

        self.blue_lower = np.array([102, 133, 88], np.uint8)
        self.blue_upper = np.array([146, 253, 157], np.uint8)

        if not self.cap.isOpened():
            self.get_logger().error(f"Não foi possível abrir a câmera no índice {self.indice_camera}!")
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(0.033, self.process_frame_callback)

    # ---------------- LIDAR ----------------
    def scan_callback(self, msg):
        if self.usar_tf and self.tf_lidar is None:
            self.tf_lidar = buscar_tf_lidar(self.tf_buffer, self.base_frame, msg.header.frame_id)
            if self.tf_lidar is None:
                self.get_logger().warn(
                    f"Sem TF {self.base_frame} <- {msg.header.frame_id}; usando lidar_offset_deg/lidar_invertido.",
                    throttle_duration_sec=5.0)
            else:
                off, inv = resumo_tf(self.tf_lidar[0])
                self.get_logger().info(f"TF do lidar carregado: equivale a offset={off:.1f}° invertido={inv}")
        yaw = estimar_yaw_mesa(msg, self.setor, self.dist_min, self.dist_max, self.min_pontos,
                               tf_lidar=self.tf_lidar, offset=self.lidar_offset,
                               invertido=self.lidar_invertido)
        if yaw is not None and abs(yaw) > self.yaw_max:
            self.get_logger().warn(f"Lidar: yaw {np.degrees(yaw):.1f}° acima do limite, descartado.",
                                   throttle_duration_sec=2.0)
            yaw = None
        self.yaw_mesa = yaw
        self.yaw_stamp = self.get_clock().now().nanoseconds / 1e9
        if self.yaw_mesa is None:
            self.get_logger().warn("Lidar: borda da mesa não encontrada no setor da frente.",
                                   throttle_duration_sec=2.0)

    def yaw_atual(self):
        """Yaw em relação à mesa, ou None se não tiver leitura recente/confiável."""
        if not self.usar_lidar or self.yaw_mesa is None:
            return None
        if self.get_clock().now().nanoseconds / 1e9 - self.yaw_stamp > 0.5:
            return None
        return self.yaw_mesa

    def correcao_yaw(self):
        yaw = self.yaw_atual()
        if yaw is None or abs(yaw) < self.zona_morta:
            return 0.0
        return float(np.clip(-self.sinal_yaw * self.k_yaw * yaw, -self.max_wz, self.max_wz))

    def publicar_cmd(self, cmd, corrigir_yaw=True):
        """Publica cmd_vel e integra o deslocamento lateral (vy fica valendo até o próximo comando).

        corrigir_yaw=True soma a correção do lidar no angular.z. Use False no ÚLTIMO comando
        antes de a visão largar o robô (senão a base pode ficar girando com o último comando).
        """
        if corrigir_yaw and cmd.angular.z == 0.0:
            cmd.angular.z = self.correcao_yaw()
        agora = self.get_clock().now().nanoseconds / 1e9
        if self.last_cmd_t is not None:
            self.desloc_y += self.last_vy * (agora - self.last_cmd_t)
        self.last_cmd_t = agora
        self.last_vy = cmd.linear.y
        self.cmd_vel_pub.publish(cmd)

    def zerar_origem(self):
        """Marca a posição atual como início da mesa."""
        self.publicar_cmd(Twist())
        self.desloc_y = 0.0
        self.time_spent_searching = 0.0

    def precision_status_callback(self, msg):
        if self.state == "WAIT_ARM":
            self.arm_result = msg.data

    def target_callback(self, msg):
        data = msg.data
        mode, target = data.split(":", 1) if ":" in data else ("pick", data)

        if mode not in self.profiles:
            self.get_logger().error(f"Modo desconhecido: {data}")
            return

        self.mode = mode
        self.vision_active = True
        self.set_detection_led(False)
        self.cube_aligned = False
        self.hits = 0
        self.prev_c = None
        self.last_seen = self.get_clock().now().nanoseconds / 1e9

        if mode == "container":
            self.container_color = target.replace("container-", "")
            if self.desloc_y > 0.01:
                self.state = "RETURNING"
                self.return_start_time = self.last_seen
            else:
                self.zerar_origem()
                self.state = "SEARCHING"
                self.search_start_time = self.last_seen
            self.get_logger().info(f"Modo CONTAINER: cor {self.container_color}")
            return

        elif mode == "precisao":
            self.current_target_id = -1
            if self.catalogo_completo():
                self.avancar_alvo_precisao()
                self.state = "RETURNING"
                self.return_start_time = self.last_seen
        
            else:
                self.tag_save = True
                self.zerar_origem()   # catálogo começa no início da mesa
                self.state = "SEARCHING"
                self.search_start_time = self.last_seen

            self.get_logger().info(f"Modo {mode.upper()}")
            return

        if not self.mode == "precisao":
            try:
                if "-" in target:
                    id, cor = target.split("-", 1)
                else:
                    id, cor = target, None
                self.current_target_id = int(id.replace("tag", ""))
                self.current_target_color = cor
            except ValueError:
                self.get_logger().error(f"Falha ao extrair ID da tag: {target}")
                # ALTERADO: avisa a ação que falhou, em vez de ficar travado procurando a tag -1
                self.vision_active = False
                self.state = "IDLE"
                self.status_pub.publish(Bool(data=False))
                return

        tempo_ocioso = self.last_seen - self.last_vision_usage
        if self.desloc_y > 0.01 and tempo_ocioso < 80.0:
            self.state = "RETURNING"
            self.return_start_time = self.last_seen
            self.last_vision_usage = 0
        else:
            self.zerar_origem()
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
            if tempo_retornando < 0:
                self.publicar_cmd(Twist())   # parado durante a pausa
                self.cap.grab()
                return
            # Volta pela distância lateral acumulada (busca + alinhamento), não só pelo tempo de busca
            if self.desloc_y > -self.margem_retorno + self.tol_retorno:
                cmd.linear.y = -0.07 # Volta para a esquerda (negativo)
                self.publicar_cmd(cmd)
                self.get_logger().info(f"Retornando à base... faltam {self.desloc_y + self.margem_retorno:.3f} m", throttle_duration_sec=0.5)
                self.cap.grab() # Limpa o buffer
                return
            else:
                # Chegou na base! Entra no estado de ESPERA
                self.zerar_origem()

                self.state = "WAITING"
                self.wait_start = agora
                self.get_logger().info(f"Retorno concluído. Aguardando {self.wait_time}s para estabilizar...")
                return
                
        # --- MÁQUINA DE ESTADOS: ESPERA (NOVO) ---
        elif self.state == "WAITING":
            if agora - self.wait_start < self.wait_time:
                cmd.linear.y = 0.0
                self.publicar_cmd(cmd)
                self.cap.grab() # Limpa o buffer
                return
            else:
                # A espera também é usada depois de catalogar os alvos de precisão.
                self.state = "SEARCHING"
                self.search_start_time = agora
                self.get_logger().info(f"Espera concluída. Iniciando busca pela Tag {self.current_target_id}.")
                # Não damos return aqui para ele fluir direto para a leitura do frame logo abaixo

        # Daqui pra frente, estamos em SEARCHING ou ALIGNING

        elif self.state == "WAIT_ARM":
            self.cap.grab()
            if self.arm_result is None:
                return
            ok, self.arm_result = self.arm_result, None

            if ok and self.current_fase == "get" and not self.primeiro_do_ciclo:
                self.current_fase = "put"
                self.arm_result = None
                slot_cheio = 2 if self.current_slot == 1 else 1
                self.info_pub.publish(String(data=f"put-slot{slot_cheio}"))
                return

            # ainda tem cubo pra organizar
            if ok and self.avancar_alvo_precisao():
                self.slot_toggle = 2 if self.slot_toggle == 1 else 1
                self.cube_aligned = False
                self.hits = 0
                self.last_seen = agora
                self.state = "RETURNING"
                self.return_start_time = agora
                return

            # counter == 5 e o put terminou (ou o braço falhou): fim da ação
            self.status_pub.publish(Bool(data=ok))
            self.vision_active = False
            self.state = "IDLE"
            self.current_fase = None
            self.counter = 0
            self.slot_toggle = 1
            self.realocable = []
            self.cycle_start = None
            self.end_pos = None
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.cap.grab()
            return

        display_frame = frame.copy()

        if self.mode == "container":
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

            if confirmado:
                self.set_detection_led(True)
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
                self.publicar_cmd(Twist())

            # --- nada visto ---
            elif self.state == "SEARCHING":
                cmd.linear.y = 0.07 * self.direction
                self.publicar_cmd(cmd)

            elif self.state == "ALIGNING" and (agora - self.last_seen) > self.lost_timeout:
                self.set_detection_led(False)
                self.state = "SEARCHING"
                self.search_start_time = agora

        else:
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
            aligned_msg = Bool()

            if self.mode == "precisao":
                if self.tag_save:
                    for d in detections:
                        tag_id = d.tag_id
                        if tag_id in self.realocable:
                            continue

                        if tag_id < 100:
                            self.realocable.insert(0, tag_id)
                        elif tag_id in self.slot_tags and self.indice_nula() is None:
                            # Tag nula: posição vazia. Entra no catálogo como uma posição.
                            self.realocable.insert(0, tag_id)
                            self.get_logger().info(f"Tag nula {tag_id} catalogada.")

                    if self.catalogo_completo():
                        self.tag_save = False
                        nula = self.indice_nula()
                        if self.slot_tags[nula] != self.realocable[nula]:
                            self.get_logger().warn(
                                f"Tag nula {self.realocable[nula]} apareceu no índice {nula} do catálogo, "
                                f"mas sua posição fixa seria {self.slot_tags.index(self.realocable[nula])}. "
                                f"Catálogo: {self.realocable}"
                            )
                        if self.state == "SEARCHING":
                            if not self.avancar_alvo_precisao():
                                self.get_logger().error("Não há cubos fora de ordem para reorganizar.")
                                self.publicar_cmd(Twist(), corrigir_yaw=False)   # para de andar/girar
                                self.vision_active = False
                                self.state = "IDLE"
                                self.status_pub.publish(Bool(data=False))
                                return
                            self.time_spent_searching = agora - self.search_start_time
                            self.state = "RETURNING"
                            self.return_start_time = agora + 2.0
                            self.get_logger().info(
                                f"Catálogo concluído: {self.realocable}. "
                                f"Aguardando {self.wait_time}s antes de iniciar a reorganização."
                            )
                    else:
                        cmd.linear.y = 0.07
                        self.publicar_cmd(cmd)
                    return
    
                '''# proximo vai ser o indice q ele deve ficar
                #next alvo da pra ficar sempre a mesma coisa, mas como eu vou atualizar alvo? alignment?
                else:
                    if self.current_fase == "get":
                        self.current_fase = "put"
                        msg = String()
                        msg.data = f"{self.current_fase}-slot{self.current_slot}"
                        self.info_pub.publish(msg)'''

            
            alvo = next((det for det in detections if det.tag_id == self.current_target_id), None)
    
            if alvo is not None:
                self.set_detection_led(True)
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
                x1 = int(min(alvo.corners[:, 0]))
                x2 = int(max(alvo.corners[:, 0]))
                y1 = int(min(alvo.corners[:, 1]))
                y2 = int(max(alvo.corners[:, 1]))

                frame_height, frame_width = frame.shape[:2]
                roi_side = max(x2 - x1, y2 - y1)   # ajuste conforme o tamanho do cubo na imagem
                gap = 5                            # px abaixo da tag

                cx_tag = (x1 + x2) // 2
                roi_left = max(0, cx_tag - roi_side // 2)
                roi_right = min(frame_width, roi_left + roi_side)
                roi_top = min(frame_height, y2 + gap)
                roi_bottom = min(frame_height, roi_top + roi_side)

                if roi_right <= roi_left or roi_bottom <= roi_top:
                    return

                roi = display_frame[roi_top:roi_bottom, roi_left:roi_right]

                cv2.rectangle(display_frame, (roi_left, roi_top), (roi_right, roi_bottom), (255, 0, 255), 2)
                self.get_logger().info(f"ROI=({roi_left},{roi_top})-({roi_right},{roi_bottom}) lado={roi_side}px cor={self.current_target_color}", throttle_duration_sec=0.5)

                # ALTERADO: só confere a cor se ela foi informada (ex.: "tag1-blue")
                if (self.mode != "precisao" and self.current_target_color is not None
                        and self.color_detection(roi, self.current_target_color) is None):
                    self.state = "SEARCHING"
                    cmd.linear.y = 0.07 * self.direction
                    self.publicar_cmd(cmd)
                    self.get_logger().info("Buscando tag para a direita...", throttle_duration_sec=1.0)
                    
                    return
                
                self.get_logger().info("Cubo detectado!")
                self.get_logger().info("Alinhando com o cubo de cor correta")
                self.alignment(tag_id, x_m, y_m, z_m, **self.profiles[self.mode])
                aligned_msg.data = self.cube_aligned
                self.aligned_pub.publish(aligned_msg)
        
    
            elif self.state == "SEARCHING":
                # Não achou a tag ainda, continua indo para a direita
                cmd.linear.y = 0.07 * self.direction
                self.publicar_cmd(cmd)
                self.get_logger().info("Buscando tag para a direita...", throttle_duration_sec=1.0)
                
            elif self.state == "ALIGNING":
                perdido = self.last_seen is None or (agora - self.last_seen) > self.lost_timeout
                if perdido:
                    self.set_detection_led(False)
                    # Se perdeu durante o alinhamento, volta pra busca
                    self.state = "SEARCHING"
                    # Reativa o contador de tempo de busca a partir de onde parou
                    self.search_start_time = agora - self.time_spent_searching
                    self.get_logger().info("Alvo perdido durante alinhamento. Retomando busca para a direita...", throttle_duration_sec=1.0)
    

        #cv2.imshow("Deteccao de Cubos - UnbDroid", display_frame)
        #cv2.waitKey(1)
        return

    def container_pixels_to_xyz(self, w_px, h_px, cx_px, cy_px):
        lado_px = max(w_px, h_px)
        if lado_px < 1:
            return None

        z = self.fx * self.container_long_side / lado_px
        x = (cx_px - self.cx0) * z / self.fx
        y = (cy_px - self.cy0) * z / self.fy
        return x, y, z


    def indice_nula(self):
        """Índice, no catálogo, da posição vazia (tag > 100). None se não houver."""
        return next((i for i, t in enumerate(self.realocable) if t > 100), None)

    def catalogo_completo(self):
        return len(self.realocable) >= self.n_posicoes and self.indice_nula() is not None

    def avancar_alvo_precisao(self):
        if self.counter == 0:
            
            cubos = sorted(t for t in self.realocable if t < 100)
            self.ordenada = cubos + [self.slot_tags[self.pos_vazia_final]]
            self.cycle_start = None

        # novo ciclo: primeira chamada ou put final feito
        if self.cycle_start is None or self.current_target_id == self.last_id:
            fora = [i for i, t in enumerate(self.realocable) if self.ordenada[i] != t] # conferir quais não estão organizados
            if not fora:
                return False

            vazia = self.indice_nula()
            if vazia != self.pos_vazia_final:
                # Ciclo da vaga: começa pelo cubo que está na 107 e termina
                # colocando o último cubo da cadeia na posição vazia atual.
                self.cycle_start = self.pos_vazia_final
                self.end_pos = vazia
            else:
                # Vaga já está na 107: ciclo normal, termina na posição de onde saiu o 1º cubo.
                self.cycle_start = fora[-1]
                self.end_pos = self.cycle_start

            self.last_id = self.slot_tags[self.end_pos]
            self.current_target_id = self.realocable[self.cycle_start]
            self.primeiro_do_ciclo = True
        else:
            self.primeiro_do_ciclo = False
            dest = self.ordenada.index(self.current_target_id)
            if dest == self.end_pos:
                self.realocable[dest] = self.current_target_id
                if self.cycle_start != self.end_pos:
                    # o cubo de partida saiu da 107: agora ela é a vaga
                    self.realocable[self.cycle_start] = self.slot_tags[self.cycle_start]
                self.current_target_id = self.last_id
            else:
                prox = self.realocable[dest]
                self.realocable[dest] = self.current_target_id
                self.current_target_id = prox

        self.counter += 1
        self.get_logger().info(
            f"[avancar] counter={self.counter} alvo={self.current_target_id} "
            f"last_id={self.last_id} mesa={self.realocable} ordenada={self.ordenada}"
        )
        return True


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

        if self.mode == "pick":
            num = 10
        else:
            num = 1000

        if cv2.countNonZero(mask) < num:
            self.get_logger().info(f'Mask {cor} menor que {num}.', throttle_duration_sec=1.0)
            return None

        # Reduz ruído
        kernel_open = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        kernel_close = np.ones((9, 9), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

       
        contours = [c for c in contours if cv2.contourArea(c) > num]

        if len(contours) == 0:
            self.get_logger().info(f'Contours == 0', throttle_duration_sec=1.0)
            return None

        # Seleciona o maior contorno
        contour = max(contours, key=cv2.contourArea)

        x, y, w, h = cv2.boundingRect(contour)

        cx = x + w // 2
        cy = y + h // 2

        centro = (cx, cy)

        #cv2.imshow("Mascara Container", mask)

        return x, y, w, h, cx, cy


    def alignment(self, tag_id, x, y, z, target_z = 0.412, x_offset = 0.10):
        tol_x = 0.012
        tol_z = 0.03    
        #tol_yaw = 0.07

        k_x = 0.42
        k_z = 0.3 
        #k_yaw = 0.3       
        max_speed = self.max_speed

        if tag_id >= 100:
            self.get_logger().info("alterando z para 60")
            target_z = 0.45 # testar isso aq

        erro_x = -x + x_offset
        erro_z = z - target_z
        #erro_yaw = yaw

        cmd = Twist()

        aligned_x = abs(erro_x) <= tol_x
        aligned_z = abs(erro_z) <= tol_z
        yaw = self.yaw_atual()
        # Sem leitura do lidar não trava o alinhamento (só avisa)
        aligned_yaw = yaw is None or abs(yaw) <= self.tol_yaw
        if yaw is None and self.usar_lidar:
            self.get_logger().warn("Alinhando sem yaw do lidar.", throttle_duration_sec=2.0)

        if aligned_x and aligned_z and aligned_yaw:
            self.cube_aligned = True

            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

            self.publicar_cmd(cmd, corrigir_yaw=False)   # parada final: sem giro

            self.get_logger().info(f"Tag ID {tag_id} ALINHADA! Robô parado.")
            self.last_vision_usage = self.get_clock().now().nanoseconds / 1e9

            if self.mode == "precisao":
                self.current_fase = "get"
                self.current_slot = self.slot_toggle
                self.arm_result = None
                self.state = "WAIT_ARM"
                if self.current_target_id == self.last_id:
                    self.current_fase = "put"
                    slot_cheio = 2 if self.current_slot == 1 else 1    # guarda o último cubo
                    self.info_pub.publish(String(data=f"{self.current_fase}-slot{slot_cheio}"))
                else:
                    self.current_fase = "get"
                    self.info_pub.publish(String(data=f"{self.current_fase}-slot{self.current_slot}"))


                self.get_logger().info(f'Alinhou a precisão: x={x:.3f} z={z:.3f} erro_x={erro_x:.3f} erro_z={erro_z:.3f}')

            else:
                self.vision_active = False
                self.state = "IDLE"
                status_msg = Bool()
                status_msg.data = True
                self.status_pub.publish(status_msg)

            self.get_logger().info(f"PAROU: x={x:.3f} z={z:.3f} erro_x={erro_x:.3f} erro_z={erro_z:.3f}")
            return True

        if not aligned_x:
            cmd.linear.y = float(np.clip(k_x * erro_x, -max_speed, max_speed))

        if not aligned_z:
            cmd.linear.x = float(np.clip(k_z * erro_z, -max_speed, max_speed))

        self.publicar_cmd(cmd)
        yaw_txt = "--" if yaw is None else f"{np.degrees(yaw):.1f}°"
        self.get_logger().info(f"Alinhando Tag {tag_id} -> CmdVel: vx={cmd.linear.x:.2f}, vy={cmd.linear.y:.2f}, "
                               f"wz={cmd.angular.z:.2f} | yaw={yaw_txt}", throttle_duration_sec=0.2)


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


    def set_detection_led(self, state):
        if self.detection_led_state == state:
            return    
        msg = Bool()
        msg.data = state
        self.detection_led_pub.publish(msg)
        self.detection_led_state = state

        if state:
            self.get_logger().info("LED de detecção LIGADO")
        else:
            self.get_logger().info("LED de detecção DESLIGADO")


    def __del__(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()



def quat_para_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def buscar_tf_lidar(tf_buffer, base_frame, laser_frame):
    """(R, t) que leva pontos do frame do lidar para o base_frame, ou None."""
    import rclpy.time
    try:
        tr = tf_buffer.lookup_transform(base_frame, laser_frame, rclpy.time.Time())
    except Exception:
        return None
    t = tr.transform.translation
    return quat_para_R(tr.transform.rotation), np.array([t.x, t.y, t.z])


def resumo_tf(R):
    """Traduz o TF para o par (offset em graus, invertido) usado no modo manual."""
    inv = R[2, 2] < 0                       # eixo z do lidar apontando para baixo
    psi = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return (-psi if inv else psi), bool(inv)


def pontos_no_robo(msg, dist_min, dist_max, tf_lidar=None, offset=0.0, invertido=False):
    """Pontos válidos do scan em x/y do robô (x para a frente, y para a esquerda)."""
    r = np.asarray(msg.ranges, dtype=float)
    cru = msg.angle_min + np.arange(len(r)) * msg.angle_increment
    v = np.isfinite(r) & (r > max(msg.range_min, dist_min)) & (r < min(msg.range_max, dist_max))
    r, cru = r[v], cru[v]
    if tf_lidar is not None:
        R, t = tf_lidar
        p = np.stack([r * np.cos(cru), r * np.sin(cru), np.zeros_like(r)])
        q = R @ p + t[:, None]
        x, y = q[0], q[1]
    else:
        ang = cru + offset
        if invertido:
            ang = -ang
        x, y = r * np.cos(ang), r * np.sin(ang)
    return x, y


def estimar_yaw_mesa(msg, setor, dist_min, dist_max, min_pontos, tf_lidar=None, offset=0.0, invertido=False):
    """Ajusta uma reta nos pontos do lidar à frente do robô e devolve o ângulo dela.

    Convenção ROS: x para a frente, y para a esquerda, ângulo positivo = anti-horário.
    Retorna o yaw do robô em relação à borda (rad, + = robô girado no sentido anti-horário),
    ou None se não houver pontos suficientes formando uma reta.
    """
    x, y = pontos_no_robo(msg, dist_min, dist_max, tf_lidar, offset, invertido)
    m = (x > 0) & (np.abs(np.arctan2(y, x)) < setor)
    if m.sum() < min_pontos:
        return None
    x, y = x[m], y[m]
    bom = reta_ransac(x, y)
    if bom is None or bom.sum() < min_pontos:
        return None
    # borda da mesa: x = a*y + b  (sem singularidade quando o robô está de frente)
    a, b = np.polyfit(y[bom], x[bom], 1)
    return float(np.arctan(a))


def reta_ransac(x, y, tol=0.015, iteracoes=150):
    """Acha a reta com mais pontos (a borda), ignorando cubo, pernas e ruído.
    Devolve a máscara dos pontos que estão na reta."""
    n = len(x)
    if n < 2:
        return None
    rng = np.random.default_rng(0)
    i = rng.integers(0, n, iteracoes)
    j = rng.integers(0, n, iteracoes)
    ok = np.abs(y[i] - y[j]) > 0.02            # pares bem separados
    i, j = i[ok], j[ok]
    if len(i) == 0:
        return None
    a = (x[i] - x[j]) / (y[i] - y[j])
    b = x[i] - a * y[i]
    dist = np.abs(x[None, :] - (a[:, None] * y[None, :] + b[:, None])) / np.sqrt(1 + a[:, None] ** 2)
    inl = dist < tol
    return inl[int(np.argmax(inl.sum(axis=1)))]


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