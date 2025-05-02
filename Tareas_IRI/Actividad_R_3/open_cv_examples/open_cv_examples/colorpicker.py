import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')

        self.img = None
        self.hsv_image = None
        self.bridge = CvBridge()

        # Suscripción a la cámara
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10
        )

        # Timer para mostrar la imagen
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Ventana con callback de clic
        cv2.namedWindow('HSV Picker')
        cv2.setMouseCallback('HSV Picker', self.mouse_callback)

        self.get_logger().info('Nodo de detección de color iniciado.')

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.hsv_image = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        except Exception as e:
            self.get_logger().error(f'Error de conversión: {e}')

    def timer_callback(self):
        if self.img is not None:
            cv2.imshow('HSV Picker', self.img)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                self.get_logger().info("Saliendo...")
                rclpy.shutdown()
        else:
            self.get_logger().info('Esperando imagen...')

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.img is not None:
            hsv_val = self.hsv_image[y, x]
            bgr_val = self.img[y, x]
            print(f'Click en ({x}, {y})')
            print(f'BGR: {bgr_val}')
            print(f'HSV: {hsv_val}')
            cv2.circle(self.img, (x, y), 5, (0, 255, 0), -1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
