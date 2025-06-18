# camara_node.py
# Nodo ROS 2 para preprocesamiento de imagen: binarización para seguimiento de línea y compresión para detección por visión

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class FilterNode(Node):
    def __init__(self):
        super().__init__('filter_node')

        # Subscripción al tópico de video en crudo proveniente de la cámara
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            10
        )

        # Publicación de imagen binaria para el nodo de seguimiento de línea
        self.filtered_image_publisher = self.create_publisher(
            Image,
            '/filtered_image',
            10
        )

        # Publicación de imagen comprimida (JPEG) para el nodo de detección por YOLOv8
        self.compressed_image_publisher = self.create_publisher(
            CompressedImage,
            '/compressed_image',
            10
        )

        self.bridge = CvBridge()                # Conversión entre mensajes ROS y OpenCV
        self.latest_frame = None                # Última imagen RGB completa
        self.latest_header = None               # Header original del mensaje de cámara
        self.frame_size = (640, 480)            # Resolución de salida para compresión

        # Temporizador que publica la imagen comprimida a ~30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info('Filter Node Initialized')

    def image_callback(self, msg):
        """
        Procesa cada frame recibido desde la cámara:
        - Rota la imagen
        - Recorta una región de interés (ROI)
        - Convierte a escala de grises y binariza
        - Aplica operación morfológica de apertura para limpieza
        - Publica la imagen binaria en /filtered_image
        - Guarda el frame completo para compresión posterior
        """
        try:
            # Conversión del mensaje de ROS a imagen OpenCV (BGR)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.rotate(frame, cv2.ROTATE_180)  # Rotar si la cámara está montada invertida
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Redimensionar y recortar región de interés (parte inferior de la imagen)
        frame = cv2.resize(frame, (1200, 480))
        roi = frame[230:480, 200:1000]

        # Convertir a escala de grises y aplicar umbral binario inverso
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY_INV)

        # Aplicar apertura morfológica para eliminar ruido
        kernel = np.ones((3, 3), np.uint8)
        clean = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=2)

        # Almacenar el último frame completo para uso en compresión
        self.latest_frame = frame
        self.latest_header = msg.header

        # Convertir la imagen binaria a mensaje y publicar
        try:
            filtered_msg = self.bridge.cv2_to_imgmsg(clean, encoding='mono8')
            self.filtered_image_publisher.publish(filtered_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing filtered image: {e}')

    def timer_callback(self):
        """
        Comprime la última imagen RGB y la publica en /compressed_image.
        Este proceso es independiente del procesamiento binario y sirve para visión con redes neuronales.
        """
        if self.latest_frame is None:
            return

        try:
            # Redimensionar imagen y codificar a JPEG
            frame = self.latest_frame.copy()
            frame_resized = cv2.resize(frame, self.frame_size)
            _, buffer = cv2.imencode('.jpg', frame_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 50])

            # Crear mensaje ROS con la imagen comprimida
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.format = "jpeg"
            compressed_msg.data = np.array(buffer).tobytes()

            # Publicar imagen comprimida
            self.compressed_image_publisher.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f'Error en timer_callback: {e}')

    def destroy_node(self):
        """
        Función de limpieza al cerrar el nodo.
        """
        super().destroy_node()


def main(args=None):
    """
    Función principal para inicializar el nodo y ejecutar su ciclo.
    """
    rclpy.init(args=args)
    node = FilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

