import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
import cv2
from cv_bridge import CvBridge
import numpy as np

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        # Subscripciones y publicaciones
        self.subscription = self.create_subscription(Image, '/filtered_image', self.image_callback, 10)
        self.error_publisher = self.create_publisher(Float32, '/error', 10)
        self.debug_image_publisher = self.create_publisher(Image, '/debug_image', 10)
        self.reset_sub = self.create_subscription(Bool, '/reset_line', self.reset_callback, 10)

        # Utilidades
        self.bridge = CvBridge()
        self.last_centers = [None, None, None]  # Últimas posiciones válidas de los tres centroides
        self.in_recover = False  # Bandera para modo recuperación
        self.last_error = 0.0  # Último error válido publicado

        # Grabación de video (debug)
        self.video_writer = None
        self.frame_size = (1200, 250)
        self.fps = 20.0
        self.video_filename = 'video_d.avi'

        self.get_logger().info('Line Follower Node Initialized')

    def image_callback(self, msg):
        # Convierte el mensaje de imagen ROS a formato OpenCV
        try:
            binary = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Prepara imagen para visualización
        roi = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        # Encuentra contornos en la imagen binaria
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        centers = [None, None, None]  # Inicializa arreglo de centroides

        if contours:
            # Selecciona el contorno más grande
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 500:
                # Crea máscara del contorno
                mask = np.zeros_like(binary)
                cv2.drawContours(mask, [largest_contour], -1, 255, -1)

                height, width = mask.shape
                step = height // 3

                # Calcula centroides en tres regiones verticales
                for i in range(3):
                    y_start = height - (i + 1) * step
                    y_end = height - i * step
                    submask = mask[y_start:y_end, :]

                    if cv2.countNonZero(submask) > 50:
                        M = cv2.moments(submask, binaryImage=True)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"]) + y_start
                            centers[i] = (cx, cy)
                            cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)

        # Cálculo de centro de la imagen y tolerancia
        image_center = roi.shape[1] // 2
        center_margin = roi.shape[1] * 0.2

        # --- Activación de modo RECOVER si se pierde el centroide 3 cerca del centro
        if centers[2] is None and self.last_centers[2] is not None:
            last_x = self.last_centers[2][0]
            if abs(last_x - image_center) < center_margin and not self.in_recover:
                self.get_logger().warn("🚨 Centroide 3 perdido cerca del centro → Modo RECOVER")
                self.error_publisher.publish(Float32(data=2.0))  # Señal especial para el controlador
                self.in_recover = True

        # --- Evaluación de validez del nuevo dato
        use_current = True
        delta_threshold = 90  # Tolerancia al cambio brusco

        if not self.in_recover:
            if centers[1] is not None and self.last_centers[1] is not None:
                dx = abs(centers[1][0] - self.last_centers[1][0])
                dy = abs(centers[1][1] - self.last_centers[1][1])
                if dx > delta_threshold or dy > delta_threshold:
                    use_current = False
                    centers = self.last_centers.copy()

            if use_current and all(c is not None for c in centers):
                self.last_centers = centers.copy()

        # --- Visualización de líneas entre centroides
        if centers[0] and centers[1]:
            cv2.line(roi, centers[0], centers[1], (255, 0, 0), 2)
        if centers[1] and centers[2]:
            cv2.line(roi, centers[1], centers[2], (255, 0, 0), 2)

        # === Publicación de error según el modo de operación ===
        if self.in_recover:
            # Modo recuperación usa centroide 2 o 1 (si disponibles)
            recover_source = None
            label = ""
            if centers[1] is not None:
                recover_source = centers[1]
                label = "2"
            elif centers[0] is not None:
                recover_source = centers[0]
                label = "1"

            if recover_source:
                recover_error = float(recover_source[0] - image_center) / image_center
                self.get_logger().info(f'Modo recover: error centroide {label} → {recover_error:.3f}')
                self.error_publisher.publish(Float32(data=recover_error))
                self.last_error = recover_error

                # Finaliza recover si se corrige el error
                if abs(recover_error) < 0.05:
                    self.get_logger().info("Recover finalizado: error cercano a 0")
                    self.in_recover = False
                    self.last_centers = [None, None, None]
            else:
                # No hay centroides → usa último error conocido
                self.get_logger().warn("Ningún centroide activo, usando último error válido")
                self.error_publisher.publish(Float32(data=self.last_error))

        elif all(c is not None for c in centers):
            # Modo normal: publica error basado en centroide 2
            error = float(centers[1][0] - image_center) / image_center
            self.last_error = error
            self.error_publisher.publish(Float32(data=error))
        else:
            self.get_logger().warn('No se detectaron los 3 centroides completos')

        # === Guardado de video e imagen de depuración ===
        if self.video_writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(self.video_filename, fourcc, self.fps, self.frame_size)

        roi_resized = cv2.resize(roi, self.frame_size)
        self.video_writer.write(roi_resized)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(roi, encoding='bgr8')
            self.debug_image_publisher.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error al publicar imagen debug: {e}')

    def reset_callback(self, msg):
        # Reinicia el estado del seguidor de línea si se recibe un reset
        if msg.data:
            self.last_centers = [None, None, None]
            self.in_recover = False
            self.get_logger().info("Reset de línea recibido → reiniciando estado")

    def destroy_node(self):
        # Libera el recurso del video si está activo
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f'Video guardado como {self.video_filename}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

