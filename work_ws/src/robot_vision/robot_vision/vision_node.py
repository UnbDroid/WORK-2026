#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node


class VisionNode(Node):

    def __init__(self):
        super().__init__("vision_node")

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error(
                "NÃO FOI POSSÍVEL ABRIR A CÂMERA! Verifique a conexão."
            )
            return

        self.get_logger().info("Câmera inicializada com sucesso a 640x480!")
        self.timer = self.create_timer(0.033, self.process_frame_callback)

    def process_frame_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Falha ao receber frame da câmera.")
            return

        self.get_logger().info("Frame capturado com sucesso!", once=True)

    def __del__(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Recursos da câmera liberados.")


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()