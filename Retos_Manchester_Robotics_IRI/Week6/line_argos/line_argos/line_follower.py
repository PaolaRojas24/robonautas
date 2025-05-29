import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge
import numpy as np
import os


class LineTrackerNode(Node):
    def __init__(self):
        super().__init__('line_tracker_node')

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            10)

        self.error_publisher = self.create_publisher(Float32, '/error', 10)
        self.debug_image_publisher = self.create_publisher(Image, '/debug_image', 10)

        self.bridge = CvBridge()
        self.last_centroid_1 = None
        self.last_centers = [None, None, None]

        self.video_writer = None
        self.frame_size = (1200, 250)
        self.fps = 20.0
        self.video_filename = 'line_vid.avi'

        self.get_logger().info('Line Tracker Node Initialized')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        frame = cv2.resize(frame, (1200, 480))
        roi = frame[230:480, 200:1000]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((3, 3), np.uint8)
        clean = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=2)

        contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 500:
                mask = np.zeros_like(binary)
                cv2.drawContours(mask, [largest_contour], -1, 255, -1)

                height, width = mask.shape
                step = height // 3
                centers = []

                for i in range(3):
                    y_start = height - (i + 1) * step
                    y_end = height - i * step
                    submask = mask[y_start:y_end, :]

                    if cv2.countNonZero(submask) > 50:
                        M = cv2.moments(submask, binaryImage=True)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"]) + y_start
                            centers.append((cx, cy))
                            cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)
                            cv2.putText(roi, str(i + 1), (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        else:
                            centers.append(None)
                    else:
                        centers.append(None)

                if all(c is not None for c in centers):
                    use_current = True
                    delta_threshold = 50

                    if self.last_centers[1] is not None:
                        dx = abs(centers[1][0] - self.last_centers[1][0])
                        dy = abs(centers[1][1] - self.last_centers[1][1])
                        if dx > delta_threshold or dy > delta_threshold:
                            use_current = False
                            centers = self.last_centers.copy()
                            self.get_logger().warn("Línea actual ignorada, se usa línea anterior")

                    if use_current:
                        self.last_centers = centers.copy()

                    cv2.line(roi, centers[0], centers[1], (255, 0, 0), 2)
                    cv2.line(roi, centers[1], centers[2], (255, 0, 0), 2)

                    image_center = roi.shape[1] // 2
                    cv2.line(roi, (image_center, 0), (image_center, roi.shape[0]), (0, 255, 255), 2)

                    cx_predictivo = int(0.7 * centers[1][0] + 0.3 * centers[2][0])
                    cv2.circle(roi, (cx_predictivo, centers[1][1]), 5, (255, 255, 0), -1)
                    cv2.putText(roi, "P", (cx_predictivo + 10, centers[1][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                    error = float(centers[1][0] - image_center) / image_center
                    self.error_publisher.publish(Float32(data=error))
                    self.get_logger().info(f'Error publicado: {error:.3f}')

                    self.last_centroid_1 = centers[0]
                else:
                    self.get_logger().warn('No se detectaron los 3 centroides completos')
                    if self.last_centroid_1:
                        self.get_logger().info(f'Usando último centroide 1: {self.last_centroid_1}')
            else:
                self.get_logger().warn('Área del contorno muy pequeña')
        else:
            self.get_logger().warn('No se detectó ningún contorno')

        if 'largest_contour' in locals():
            cv2.drawContours(roi, [largest_contour], -1, (0, 255, 0), 2)

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

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f'Video guardado como {self.video_filename}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
