import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class ColorDetectionNode(Node):

    def __init__(self):
        super().__init__('color_detection_node')

        # === HSV RANGES CALIBRADOS ===

        # 🔴 Rojo (dos rangos por estar en el borde del espectro HSV)
        self.lower_red1 = np.array([170, 100, 200])
        self.upper_red1 = np.array([179, 255, 255])
        self.lower_red2 = np.array([0, 100, 200])
        self.upper_red2 = np.array([5, 255, 255])

        # 🟢 Verde (ajustado para incluir verde claro)
        self.lower_green1 = np.array([60, 80, 180])
        self.upper_green1 = np.array([75, 255, 255])
        self.lower_green2 = np.array([75, 60, 200])
        self.upper_green2 = np.array([95, 255, 255])

        # 🟡 Amarillo (dividido en dos rangos para más robustez)
        self.lower_yellow1 = np.array([18, 40, 200])
        self.upper_yellow1 = np.array([26, 255, 255])
        self.lower_yellow2 = np.array([26, 40, 200])
        self.upper_yellow2 = np.array([32, 255, 255])

        self.bridge = CvBridge()
        self.img = None

        # Suscriptor
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10
        )

        # Publicadores
        self.pub_red = self.create_publisher(Image, '/image_processing/imagered', 10)
        self.pub_green = self.create_publisher(Image, '/image_processing/imagegreen', 10)
        self.pub_yellow = self.create_publisher(Image, '/image_processing/imageyellow', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Color Detection Node has started!')

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def timer_callback(self):
        if self.img is None:
            self.get_logger().info('Waiting for image data...')
            return

        image = self.img.copy()
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        # Red mask (dos rangos unidos)
        red_mask1 = cv.inRange(hsv_image, self.lower_red1, self.upper_red1)
        red_mask2 = cv.inRange(hsv_image, self.lower_red2, self.upper_red2)
        red_mask = cv.bitwise_or(red_mask1, red_mask2)

        # Green mask (dos rangos unidos)
        green_mask1 = cv.inRange(hsv_image, self.lower_green1, self.upper_green1)
        green_mask2 = cv.inRange(hsv_image, self.lower_green2, self.upper_green2)
        green_mask = cv.bitwise_or(green_mask1, green_mask2)

        # Yellow mask (dos rangos unidos)
        yellow_mask1 = cv.inRange(hsv_image, self.lower_yellow1, self.upper_yellow1)
        yellow_mask2 = cv.inRange(hsv_image, self.lower_yellow2, self.upper_yellow2)
        yellow_mask = cv.bitwise_or(yellow_mask1, yellow_mask2)

        # Limpieza morfológica (reduce ruido)
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv.morphologyEx(red_mask, cv.MORPH_OPEN, kernel)
        green_mask = cv.morphologyEx(green_mask, cv.MORPH_OPEN, kernel)
        yellow_mask = cv.morphologyEx(yellow_mask, cv.MORPH_OPEN, kernel)

        # Aplicar máscara sobre la imagen original para conservar el color
        red_result = cv.bitwise_and(image, image, mask=red_mask)
        green_result = cv.bitwise_and(image, image, mask=green_mask)
        yellow_result = cv.bitwise_and(image, image, mask=yellow_mask)

        # Publicar resultados con color
        self.pub_red.publish(self.bridge.cv2_to_imgmsg(red_result, encoding="bgr8"))
        self.pub_green.publish(self.bridge.cv2_to_imgmsg(green_result, encoding="bgr8"))
        self.pub_yellow.publish(self.bridge.cv2_to_imgmsg(yellow_result, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
