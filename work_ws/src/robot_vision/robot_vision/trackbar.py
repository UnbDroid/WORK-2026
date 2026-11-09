import cv2
import numpy as np
import apriltag
import rclpy
from rclpy.node import Node


# ==============================================================================
# 1. CLASSE DE DETECÇÃO DE CORES
# ==============================================================================
# ==============================================================================
# 1. CLASSE DE DETECÇÃO DE CORES
# ==============================================================================
class ColorDetector:
    def __init__(self):
        self.window_red = "Calibracao HSV (Vermelho)"
        self.window_blue = "Calibracao HSV (Azul)"

        cv2.namedWindow(self.window_red, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self.window_blue, cv2.WINDOW_AUTOSIZE)

        def onChange(val):
            pass

        # Criação dos sliders para o Vermelho
        cv2.createTrackbar("Min Hue", self.window_red, 0, 179, onChange)
        cv2.createTrackbar("Max Hue", self.window_red, 10, 179, onChange)
        cv2.createTrackbar("Min Sat", self.window_red, 120, 255, onChange)
        cv2.createTrackbar("Max Sat", self.window_red, 255, 255, onChange)
        cv2.createTrackbar("Min Val", self.window_red, 70, 255, onChange)
        cv2.createTrackbar("Max Val", self.window_red, 255, 255, onChange)

        # Criação dos sliders para o Azul
        cv2.createTrackbar("Min Hue", self.window_blue, 90, 179, onChange)
        cv2.createTrackbar("Max Hue", self.window_blue, 130, 179, onChange)
        cv2.createTrackbar("Min Sat", self.window_blue, 50, 255, onChange)
        cv2.createTrackbar("Max Sat", self.window_blue, 255, 255, onChange)
        cv2.createTrackbar("Min Val", self.window_blue, 50, 255, onChange)
        cv2.createTrackbar("Max Val", self.window_blue, 255, 255, onChange)

        self.kernel = np.ones((3, 3), np.uint8)

    def get_trackbar_limits(self, window_name):
        h_min = cv2.getTrackbarPos("Min Hue", window_name)
        h_max = cv2.getTrackbarPos("Max Hue", window_name)
        s_min = cv2.getTrackbarPos("Min Sat", window_name)
        s_max = cv2.getTrackbarPos("Max Sat", window_name)
        v_min = cv2.getTrackbarPos("Min Val", window_name)
        v_max = cv2.getTrackbarPos("Max Val", window_name)

        # Garante que o valor mínimo nunca supere o máximo na janela específica
        if h_min > h_max:
            cv2.setTrackbarPos("Max Hue", window_name, h_min)
            h_max = h_min
        if s_min > s_max:
            cv2.setTrackbarPos("Max Sat", window_name, s_min)
            s_max = s_min
        if v_min > v_max:
            cv2.setTrackbarPos("Max Val", window_name, v_min)
            v_max = v_min

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        return lower, upper

        # dentro de ColorDetector
    def detect(self, frame):
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        rl, ru = self.get_trackbar_limits(self.window_red)
        red_mask = cv2.bitwise_or(
            cv2.inRange(img_hsv, rl, ru),
            cv2.inRange(img_hsv, np.array([170, rl[1], rl[2]]),
                                 np.array([179, ru[1], ru[2]])))

        bl, bu = self.get_trackbar_limits(self.window_blue)
        blue_mask = cv2.inRange(img_hsv, bl, bu)

        k_open = np.ones((3, 3), np.uint8)
        k_close = np.ones((9, 9), np.uint8)
        results = []
        for mask, name in [(red_mask, "red"), (blue_mask, "blue")]:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close, iterations=2)
            cv2.imshow(f"Mascara {name}", mask)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) > 500]
            if contours:
                c = max(contours, key=cv2.contourArea)
                results.append((name, cv2.boundingRect(c), cv2.contourArea(c)))
        return results

    def print_values(self):
        rl, ru = self.get_trackbar_limits(self.window_red)
        bl, bu = self.get_trackbar_limits(self.window_blue)
        print(f"self.red_lower  = np.array({rl.tolist()}, np.uint8)")
        print(f"self.red_upper  = np.array({ru.tolist()}, np.uint8)")
        print(f"self.blue_lower = np.array({bl.tolist()}, np.uint8)")
        print(f"self.blue_upper = np.array({bu.tolist()}, np.uint8)")


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('hsv_calibration')
        self.detector = ColorDetector()
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)  # mesmo índice do vision_node real
        if not self.cap.isOpened():
            self.get_logger().error("Não foi possível abrir a câmera!")
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.create_timer(0.033, self.loop)
        self.get_logger().info("Aperte 'p' numa janela do OpenCV para imprimir os valores.")

    def loop(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return
        display = frame.copy()
        for name, (x, y, w, h), area in self.detector.detect(frame):
            cor = (0, 0, 255) if name == "red" else (255, 0, 0)
            cv2.rectangle(display, (x, y), (x + w, y + h), cor, 2)
            cv2.putText(display, f"{name} area={int(area)}", (x, max(y - 10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, cor, 2)
        cv2.imshow("Calibracao", display)
        if cv2.waitKey(1) & 0xFF == ord('p'):
            self.detector.print_values()

# ==============================================================================
# PONTO DE ENTRADA DO SCRIPT
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()