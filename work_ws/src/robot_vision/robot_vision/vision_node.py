#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
import apriltag
import numpy as np

from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import MultiArrayLayout
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

        self.get_logger().info("Publishers 'cube_coordinates' e 'cube_data' inicializados!")

        self.indice_camera = 0 # Índice na raspberry pi (diferente no notebook)
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

    def process_frame_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.cap.grab()
            return

        display_frame = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        coord_msg = PointStamped()
        coord_msg.header.stamp = self.get_clock().now().to_msg()
        coord_msg.header.frame_id = "webcam_link"
        
        array_msg = CubeArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = "webcam_link" 

        for det in detections:
            tag_id = det.tag_id

            camera_params = [500.0, 500.0, 320.0, 240.0] 
            pose, _, _ = self.detector.detection_pose(
                det, camera_params, tag_size=self.tag_size
            )

            x_m = pose[0][3] 
            y_m = pose[1][3] 
            z_m = pose[2][3] 

            coord_msg.point.x = float(x_m)
            coord_msg.point.y = float(y_m)
            coord_msg.point.z = float(z_m)

            self.render_preview(display_frame, det, tag_id, x_m, y_m, z_m) 

            self.coord_pub.publish(coord_msg)

            cube_msg = Cube()
            cube_msg.id = int(tag_id)
            cube_msg.waypoint = "default"  
            cube_msg.color = "cor"  
            array_msg.cubes.append(cube_msg)

            self.get_logger().info(f"Tag ID: {tag_id}: Coords: X={x_m:.5f} Y={y_m:.5f} Z={z_m:.5f} m")

            self.render_preview(display_frame, det, tag_id, pose[0][3], pose[1][3], pose[2][3])

            self.cube_alignment(tag_id, x_m, y_m, z_m)

        #cv2.imshow("Deteccao de Cubos - UnbDroid", display_frame)
        #cv2.waitKey(1)

    def color_detection(self, roi):
        hsv_frame = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(hsv_frame, self.red_lower, self.red_upper)
        blue_mask = cv2.inRange(hsv_frame, self.blue_lower, self.blue_upper)

        red_count = cv2.countNonZero(red_mask)
        blue_count = cv2.countNonZero(blue_mask)

        if red_count > blue_count:
            return "red"
        elif blue_count > red_count:
            return "blue"
        else:
            self.get_logger().error("Nenhuma cor detectada.")
            return "unknown"

    def cube_alignment(self, tag_id, x, y, z):
        target_z = 0.27      
        tol_x = 0.03           
        tol_z = 0.03           

        # pra evitar que o robo fique dando trancos na hora de se mover
        k_x = 0.4              
        k_z = 0.3              
        max_speed = 0.2        

        erro_x = -x
        erro_z = z - target_z

        cmd = Twist()

        aligned_x = abs(x) <= tol_x
        aligned_z = abs(erro_z) <= tol_z

        if aligned_x and aligned_z: # se esta alinhado para
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(f"Tag ID {tag_id} ALINHADA! Robô parado.")
            return

        # ajuste no eixo x
        if not aligned_x:
            cmd.linear.y = float(np.clip(k_x * erro_x, -max_speed, max_speed))

        # ajuste no eixo z
        if not aligned_z:
            #cmd.linear.x = float(np.clip(k_z * erro_z, -max_speed, max_speed))
            cmd.linear.x = max_speed if erro_z > 0 else -max_speed # testar com diferentes valores de vel menores

        self.get_logger().info(
                    f"Alinhando Tag {tag_id} -> CmdVel: vx={cmd.linear.x:.2f}, vy={cmd.linear.y:.2f}"
                )
        
        self.cmd_vel_pub.publish(cmd)
    
    def render_preview(self, image, detection, tag_id, x, y, z):
        """Função dedicada para desenhar os elementos gráficos na tela usando OpenCV."""

        corners = detection.corners.astype(int)
        for i in range(4):
            p1 = tuple(corners[i])
            p2 = tuple(corners[(i + 1) % 4])
            cv2.line(image, p1, p2, (0, 255, 0), 2)

        center = (int(detection.center[0]), int(detection.center[1]))
        cv2.circle(image, center, 5, (0, 0, 255), -1)
        
        texto_id = f"ID: {tag_id}"
        texto_coords = f"X:{x:.1f} Y:{y:.1f} Z:{z:.1f} m"
        texto_color = f"Cor: {"Cor"}"

        pos_id = (center[0] - 40, center[1] - 25)
        pos_coords = (center[0] - 75, center[1] - 10)
        pos_color = (center[0] - 40, center[1] + 15)

        cv2.putText(image, texto_id, pos_id, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(image, texto_coords, pos_coords, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
        cv2.putText(image, texto_color, pos_color, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)

        cv2.putText(image, texto_id, pos_id, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(image, texto_coords, pos_coords, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        cv2.putText(image, texto_color, pos_color, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # self.get_logger().info(f"Tag ID: {tag_id}, Coords: X={x:.1f} Y={y:.1f} Z={z:.1f} m")


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