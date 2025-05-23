import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()

        # Abre la cámara (puede cambiar /dev/video0 por el índice 0)
        self.cap = cv2.VideoCapture("/dev/video0")  # O el número correcto si no es el 0

        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara.')
        else:
            self.get_logger().info('Cámara abierta exitosamente.')

        # Temporizador para capturar y publicar imágenes cada 0.1 segundos (~10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('No se pudo capturar imagen de la cámara.')
            return

        # Convertir la imagen a mensaje ROS
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Imagen publicada.')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
