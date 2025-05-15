import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
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
        self.lower_red2 = np.array([0, 80, 200])
        self.upper_red2 = np.array([20, 255, 255])

        # 🟡 Amarillo (un solo rango)
        self.lower_yellow = np.array([28, 30, 180])
        self.upper_yellow = np.array([35, 255, 255])

        # 🟢 Verde (un solo rango)
        self.lower_green = np.array([70, 150, 150])
        self.upper_green = np.array([90, 255, 255])

        self.bridge = CvBridge()
        self.img = None

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10
        )

        # self.pub_mask = self.create_publisher(Image, '/image_processing/traffic_light_mask', 10)
        self.pub_color = self.create_publisher(Int32, '/color', 10)

        # Procesar cada 0.2 s ≈ 5 FPS (de 30 posibles)
        self.timer = self.create_timer(0.2, self.timer_callback)
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

        # Reducir resolución a la mitad para menos carga computacional
        image = cv.resize(self.img, (0, 0), fx=0.5, fy=0.5)
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        red_mask1 = cv.inRange(hsv_image, self.lower_red1, self.upper_red1)
        red_mask2 = cv.inRange(hsv_image, self.lower_red2, self.upper_red2)
        red_mask = cv.bitwise_or(red_mask1, red_mask2)

        yellow_mask = cv.inRange(hsv_image, self.lower_yellow, self.upper_yellow)
        green_mask = cv.inRange(hsv_image, self.lower_green, self.upper_green)

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv.morphologyEx(red_mask, cv.MORPH_OPEN, kernel)
        yellow_mask = cv.morphologyEx(yellow_mask, cv.MORPH_OPEN, kernel)
        green_mask = cv.morphologyEx(green_mask, cv.MORPH_OPEN, kernel)

        # Si quieres mantener visualización: descomenta lo siguiente
        # red_colored = cv.merge([np.zeros_like(red_mask), np.zeros_like(red_mask), red_mask])
        # yellow_colored = cv.merge([np.zeros_like(yellow_mask), np.zeros_like(yellow_mask), yellow_mask])
        # green_colored = cv.merge([np.zeros_like(green_mask), green_mask, np.zeros_like(green_mask)])
        # final_mask = cv.addWeighted(red_colored, 1.0, yellow_colored.astype(np.uint8), 1.0, 0.0)
        # final_mask = cv.addWeighted(final_mask, 1.0, green_colored, 1.0, 0.0)
        # self.pub_mask.publish(self.bridge.cv2_to_imgmsg(final_mask, encoding="bgr8"))

        red_pixels = cv.countNonZero(red_mask)
        yellow_pixels = cv.countNonZero(yellow_mask)
        green_pixels = cv.countNonZero(green_mask)

        if max(red_pixels, yellow_pixels, green_pixels) < 500:
            state = 0
        elif red_pixels > yellow_pixels and red_pixels > green_pixels:
            state = 1
        elif green_pixels > yellow_pixels:
            state = 2
        else:
            state = 3

        self.get_logger().info(f"Traffic light detected: {state}")

        msg = Int32()
        msg.data = state
        self.pub_color.publish(msg)

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
