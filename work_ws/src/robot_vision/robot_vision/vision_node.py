#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
import apriltag
import numpy as np

from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import Bool
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

        # PONTE COM O ACTION_NODE (Interruptor)
        self.vision_active = False 
        self.create_subscription(Bool, "/vision_trigger", self.trigger_callback, 10)
        self.status_pub = self.create_publisher(Bool, "/vision_status", 10)
        #self.aligned_pub = self.create_publisher(Bool, "/cube_aligned", 10)

        self.container_detection_enabled = False
        self.create_subscription(Bool, "container_detection_enabled", self.container_enabled_callback, 10)
        self.container_coord_pub = self.create_publisher(PointStamped, 'container_coordinates', 10)

        self.get_logger().info("Publishers 'cube_coordinates' e 'cube_data' inicializados!")

        self.cube_priority = [2, 3, 4]
        self.cube_aligned = False
        self.counter = 0

        self.v_busca = 0.05      # m/s

        self.direction = -1
        self.duration_start = None
        self.duration = 1.0
        self.duration_max = 6.0

        self.indice_camera = 2 # Índice na raspberry pi (diferente no notebook)
        self.cap = cv2.VideoCapture(self.indice_camera, cv2.CAP_V4L2)

        self.red_lower = np.array([136, 87, 111], np.uint8)
        self.red_upper = np.array([180, 255, 255], np.uint8)

        self.blue_lower = np.array([94, 80, 2], np.uint8)
        self.blue_upper = np.array([120, 255, 255], np.uint8)

        if not self.cap.isOpened():
            self.get_logger().error(f"Não foi possível abrir a câmera no índice {self.indice_camera}!")
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(0.033, self.process_frame_callback)

    def trigger_callback(self, msg):
        self.vision_active = msg.data
        if self.vision_active:
            self.cube_aligned = False
            self.duration_start = None
            self.get_logger().info('Visão ACORDADA pelo PDDL! Procurando cubo...')
        else:
            self.get_logger().info('Visão PAUSADA. Aguardando nova ordem.')

    def container_enabled_callback(self, msg):
        self.container_detection_enabled = msg.data
        if msg.data:
            self.get_logger().info('Detecção de contêiner habilitada.')
        else:
            self.get_logger().info('Detecção de contêiner desabilitada.')

    def process_frame_callback(self):
        # Trava para não consumir CPU enquanto viaja
        if not self.vision_active:
            self.cap.grab()
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.cap.grab()
            return

        display_frame = frame.copy()
        agora = self.get_clock().now().nanoseconds / 1e9
 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = [] if self.cube_aligned else self.detector.detect(gray)

        ids_vistos = [d.tag_id for d in detections]
        self.get_logger().info(
            f"Tags no frame: {ids_vistos} | alvo: {self.cube_priority[0]}",
            throttle_duration_sec=0.5)
        
        coord_msg = PointStamped()
        coord_msg.header.stamp = self.get_clock().now().to_msg()
        coord_msg.header.frame_id = "webcam_link"
        
        array_msg = CubeArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = "webcam_link"
 
        for det in detections:
            tag_id = det.tag_id

            if tag_id == self.cube_priority[0]:
                camera_params = [500.0, 500.0, 320.0, 240.0] 
                pose, _, _ = self.detector.detection_pose(
                    det, camera_params, tag_size=self.tag_size
                )
    
                x_m = pose[0][3] 
                y_m = pose[1][3] 
                z_m = pose[2][3] 
    
                self.duration_start = None
    
                coord_msg.point.x = float(x_m)
                coord_msg.point.y = float(y_m)
                coord_msg.point.z = float(z_m)
                self.coord_pub.publish(coord_msg)
    
                cube_msg = Cube()
                cube_msg.id = int(tag_id)
                cube_msg.waypoint = "default"  
                cube_msg.color = "cor"  
                array_msg.cubes.append(cube_msg)

                self.get_logger().info(f"Tag ID: {tag_id}: Coords: X={x_m:.5f} Y={y_m:.5f} Z={z_m:.5f} m")
                self.render_preview(display_frame, det, tag_id, x_m, y_m, z_m)
    
                self.cube_alignment(tag_id, x_m, y_m, z_m)

                cmd = Twist()  # Se está vendo o alvo, não precisa mover o robô para varrer
                #aligned_msg.data = self.cube_aligned
                #self.aligned_pub.publish(aligned_msg)


            else:
                if not self.cube_aligned:
                    cmd = self.varredura(agora)
                    self.get_logger().info("Nenhum cubo alvo detectado, varrendo", throttle_duration_sec=1.0)
            
            self.cmd_vel_pub.publish(cmd)

        if detections == [] and not self.cube_aligned:
            cmd = self.varredura(agora)
            self.get_logger().info("Nenhum cubo alvo detectado, varrendo", throttle_duration_sec=1.0)
            self.cmd_vel_pub.publish(cmd)
        
        container = None
        if self.container_detection_enabled:
            container = self.color_detection(display_frame, cor='red')

        if container is not None:
            x, y, w, h, cx, cy = container

            container_msg = PointStamped()
            container_msg.header.stamp = self.get_clock().now().to_msg()
            container_msg.header.frame_id = 'webcam_link'

            container_msg.point.x = float(cx)
            container_msg.point.y = float(cy)
            container_msg.point.z = 0.0

            self.container_coord_pub.publish(container_msg)
 
        cv2.imshow("Deteccao de Cubos - UnbDroid", display_frame)
        cv2.waitKey(1)
 
    def varredura(self, agora):
        cmd = Twist()
 
        if self.duration_start is None:
            self.duration_start = agora
            self.duration = 1.0
            self.direction = 1
 
        if agora - self.duration_start > self.duration:
            self.direction *= -1
            self.duration = min(self.duration + 1.0, self.duration_max)
            self.duration_start = agora
 
        cmd.linear.y = self.v_busca * self.direction
        return cmd

    def color_detection(self, roi, cor= "red"):
        hsv_frame = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(hsv_frame, self.red_lower, self.red_upper)
        blue_mask = cv2.inRange(hsv_frame, self.blue_lower, self.blue_upper)

        mask = red_mask if cor == "red" else blue_mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.get_logger().info(f"Nenhum contorno {cor} detectado.")
            return None

        contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contour)
        cx = x + w // 2 
        cy = y + h // 2
        centro = (cx, cy)
        contorno = x, y, w, h

        self.get_logger().info(f"Container {cor} detectado em ({cx}, {cy})")
        self.render_container_preview(roi, contorno, cor, centro)

        return x, y, w, h, cx, cy

    def cube_alignment(self, tag_id, x, y, z):
        target_z = 0.40
        tol_x = 0.02
        tol_z = 0.03    

        k_x = 0.4
        k_z = 0.4            
        max_speed = 0.05     

        erro_x = -x + 0.1
        erro_z = z - target_z

        cmd = Twist()

        aligned_x = abs(erro_x) <= tol_x
        aligned_z = abs(erro_z) <= tol_z

        if aligned_x and aligned_z: 
            self.cube_aligned = True
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(f"Tag ID {tag_id} ALINHADA! Robô parado.")
            
            # AVISA O ACTION_NODE QUE TERMINOU
            status_msg = Bool()
            status_msg.data = True
            self.status_pub.publish(status_msg)
            self.vision_active = False 
            
            return True

        if not aligned_x:
            cmd.linear.y = float(np.clip(k_x * erro_x, -max_speed, max_speed))

        if not aligned_z:
            cmd.linear.x = float(np.clip(k_z * erro_z, -max_speed, max_speed))

        self.get_logger().info(f"Alinhando Tag {tag_id} -> CmdVel: vx={cmd.linear.x:.2f}, vy={cmd.linear.y:.2f}")
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