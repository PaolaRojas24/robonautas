import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # Suscripción a imágenes comprimidas desde la cámara
        self.subscription = self.create_subscription(
            CompressedImage,
            '/compressed_image',
            self.image_callback,
            10
        )

        # Publicadores para códigos de semáforo y señales de tránsito
        self.color_publisher = self.create_publisher(Int32, '/color', 10)
        self.sign_publisher = self.create_publisher(Int32, '/sign', 10)

        self.bridge = CvBridge()

        # Carga del modelo YOLOv8
        self.model = YOLO("/home/paoro/ros2_w/src/final_argos/best.pt")
        self.get_logger().info("Modelo YOLO cargado correctamente.")

        # Parámetros de grabación de video
        self.video_writer = None
        self.frame_shape = None

        # Variables de control de detección
        self.last_detection_time = 0
        self.last_no_object_logged = False
        self.last_class_str = ""
        self.multiple_log_count = 0
        self.last_info_log_time = 0

        # Diccionario de códigos personalizados para cada clase detectada
        self.custom_class_codes = {
            "S_rojo": 1,
            "S_verde": 2,
            "S_amarillo": 3,
            "right": 4,
            "left": 5,
            "front": 6,
            "stop": 7,
            "giveway": 8,
            "work": 9
        }

        # Áreas mínimas requeridas por clase para validar detección
        self.min_areas = {
            "S_rojo": 2900,
            "S_verde": 2900,
            "S_amarillo": 3870,
            "right": 18320,
            "left": 18100,
            "front": 12000,
            "stop": 15500,
            "giveway": 19500,
            "work": 28500
        }

        # Íconos informativos por clase (solo para logging)
        self.custom_icons = {
            "S_rojo": '🔴',
            "S_verde": '🟢',
            "S_amarillo": '🟡',
            "right": '↪️',
            "left": '↩️',
            "front": '⬆️',
            "stop": '🛑',
            "giveway": '⚠️',
            "work": '🚧'
        }

        # Persistencia de detección para evitar falsos positivos
        self.persistent_detections = {}
        self.min_persistence_time = 0.5  # segundos

    def image_callback(self, msg: CompressedImage):
        current_time = time.time()

        # Decodificación de imagen comprimida
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().error("No se pudo decodificar la imagen.")
            return

        # Inicialización del video writer con la forma del primer frame
        if self.frame_shape is None:
            self.frame_shape = (frame.shape[1], frame.shape[0])
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter('bounding_box_vid.avi', fourcc, 15.0, self.frame_shape)
            self.get_logger().info(f"Iniciando grabación de video: bounding_box_vid.avi")

        # Ejecución del modelo YOLO
        infer_start = time.time()
        results = self.model(frame)[0]
        infer_end = time.time()

        # Copia para anotaciones
        annotated_frame = frame.copy()

        # Inicialización de variables de salida
        detected_classes = []
        detected_codes = []
        sign_codes = []
        color_code = 0
        current_detections = set()

        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            confidence = float(box.conf[0])
            class_name = self.model.names[cls_id]

            area = (x2 - x1) * (y2 - y1)
            icon = self.custom_icons.get(class_name, "❓")
            self.get_logger().info(f"{icon} {class_name} detectado con área: {area} píxeles²")

            # Filtrado por área mínima
            min_area = self.min_areas.get(class_name, 0)
            if area < min_area:
                self.get_logger().info(f"{icon} {class_name} ignorado por área insuficiente ({area} < {min_area})")
                continue

            current_detections.add(class_name)

            # Validar persistencia
            if class_name not in self.persistent_detections:
                self.persistent_detections[class_name] = current_time
                self.get_logger().info(f"{class_name} detectado por primera vez, esperando persistencia...")
                continue

            time_seen = current_time - self.persistent_detections[class_name]
            if time_seen < self.min_persistence_time:
                self.get_logger().info(f"{class_name} aún no confirmado ({time_seen:.2f}s < {self.min_persistence_time}s)")
                continue

            # Detección confirmada
            detected_classes.append(class_name)
            if class_name in self.custom_class_codes:
                code = self.custom_class_codes[class_name]
                detected_codes.append(code)
                if 4 <= code <= 9:
                    sign_codes.append(code)

            # Determinar color del semáforo
            if class_name == 'S_rojo':
                color_code = 1
            elif class_name == 'S_verde' and color_code == 0:
                color_code = 2
            elif class_name == 'S_amarillo' and color_code == 0:
                color_code = 3

            # Dibujar bounding box
            color = (0, 255, 0)
            if class_name == 'S_amarillo':
                color = (0, 255, 255)
            elif class_name == 'S_rojo':
                color = (0, 0, 255)

            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            label = f"{class_name} {confidence:.2f} A:{area}"
            cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Eliminar detecciones antiguas no vistas
        for prev_class in list(self.persistent_detections.keys()):
            if prev_class not in current_detections:
                del self.persistent_detections[prev_class]

        # Guardar video con anotaciones
        self.video_writer.write(annotated_frame)

        # Publicar resultados
        if not detected_classes:
            if not self.last_no_object_logged:
                self.get_logger().info("No se encuentran objetos en el área.")
                self.last_no_object_logged = True

            self.color_publisher.publish(Int32(data=0))
            self.sign_publisher.publish(Int32(data=0))
            self.last_class_str = ""
            self.multiple_log_count = 0
        else:
            self.last_no_object_logged = False
            self.color_publisher.publish(Int32(data=color_code))
            if sign_codes:
                for sign_code in sign_codes:
                    self.sign_publisher.publish(Int32(data=sign_code))
            else:
                self.sign_publisher.publish(Int32(data=0))

            class_str = ", ".join(detected_classes)
            code_str = f"[{', '.join(map(str, detected_codes))}]"

            if class_str != self.last_class_str:
                self.multiple_log_count = 3 if len(detected_classes) > 1 else 1
                self.last_detection_time = 0
                self.last_class_str = class_str

            if current_time - self.last_detection_time >= 3 or self.multiple_log_count > 0:
                self.get_logger().info(f"Detectado(s): {class_str} {code_str}")
                self.last_detection_time = current_time
                self.multiple_log_count -= 1

        # Log de tiempo de inferencia
        if current_time - self.last_info_log_time >= 1.0:
            inference_time = (infer_end - infer_start) * 1000
            self.get_logger().info(
                f"Inferencia: {inference_time:.1f} ms @ shape (1, 3, {self.frame_shape[1]}, {self.frame_shape[0]})"
            )
            self.last_info_log_time = current_time

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info("Video guardado correctamente en bounding_box_vid.avi")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

