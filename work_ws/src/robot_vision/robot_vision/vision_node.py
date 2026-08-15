#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
import apriltag

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from robot_vision.msg import Cube
from robot_vision.msg import CubeArray

class VisionNode(Node):

    def __init__(self):
        super().__init__("vision_node")

        self.tag_size_cm = 5.0  

        options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(options)

        self.coord_pub = self.create_publisher(Float32MultiArray, "cube_coordinates", 10)
        self.cube_data_pub = self.create_publisher(CubeArray, "cube_data", 10)

        self.get_logger().info("Publishers 'cube_coordinates' e 'cube_data' inicializados!")

        self.indice_camera = 3
        self.cap = cv2.VideoCapture(self.indice_camera, cv2.CAP_V4L2)

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

        coordinates_msg = Float32MultiArray()
        array_msg = CubeArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = "camera_link"

        for det in detections:
            tag_id = det.tag_id

            camera_params = [500.0, 500.0, 320.0, 240.0] 
            pose, _, _ = self.detector.detection_pose(
                det, camera_params, tag_size=self.tag_size_cm
            )

            x_m = pose[0][3] / 100.0
            y_m = pose[1][3] / 100.0
            z_m = pose[2][3] / 100.0

            coord_msg = Float32MultiArray()
            coord_msg.data = [float(x_m), float(y_m), float(z_m)]
            self.coord_pub.publish(coord_msg)

            cube_msg = Cube()
            cube_msg.id = int(tag_id)
            cube_msg.waypoint = "default"  
            cube_msg.color = "unknown"     
            array_msg.cubes.append(cube_msg)

            self.render_preview(display_frame, det, tag_id, pose[0][3], pose[1][3], pose[2][3])

        if len(array_msg.cubes) > 0:
            self.cube_data_pub.publish(array_msg)

        cv2.imshow("Deteccao de Cubos - UnbDroid", display_frame)
        cv2.waitKey(1)

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
        texto_coords = f"X:{x:.1f} Y:{y:.1f} Z:{z:.1f} cm"

        pos_id = (center[0] - 40, center[1] - 25)
        pos_coords = (center[0] - 75, center[1] - 10)

        cv2.putText(image, texto_id, pos_id, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(image, texto_coords, pos_coords, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)

        cv2.putText(image, texto_id, pos_id, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(image, texto_coords, pos_coords, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

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