import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

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
        self.last_centers = {'left': None, 'center': None, 'right': None}
        self.get_logger().info('Line Tracker Node Initialized')

    def actualizar_labels(self, centers):
        labels = {'left': None, 'center': None, 'right': None}
        centers.sort(key=lambda c: c[0])

        if len(centers) == 3:
            labels['left'], labels['center'], labels['right'] = centers
        elif len(centers) == 2:
            for c in centers:
                if self.last_centers['left'] is None:
                    labels['left'] = c
                    self.last_centers['left'] = c
                elif abs(c[0] - self.last_centers['left'][0]) < 40:
                    labels['left'] = c
                elif self.last_centers['right'] is None:
                    labels['right'] = c
                    self.last_centers['right'] = c
                elif abs(c[0] - self.last_centers['right'][0]) < 40:
                    labels['right'] = c
                else:
                    labels['center'] = c
        elif len(centers) == 1:
            c = centers[0]
            min_dist = float('inf')
            closest_label = None
            for label, last in self.last_centers.items():
                if last is not None:
                    dist = abs(c[0] - last[0])
                    if dist < min_dist:
                        min_dist = dist
                        closest_label = label
            if closest_label:
                labels[closest_label] = c

        for label in ['left', 'center', 'right']:
            if labels[label] is None:
                labels[label] = self.last_centers[label]
            else:
                self.last_centers[label] = labels[label]

        return labels

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        frame = cv2.resize(frame, (1200, 480))
        roi = frame[0:300, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((3, 3), np.uint8)
        clean = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=2)

        contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_centers = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    detected_centers.append((cx, cy))
                    cv2.circle(roi, (cx, cy), 4, (100, 200, 0), -1)
                cv2.drawContours(roi, [cnt], -1, (0, 255, 0), 2)

        labels = self.actualizar_labels(detected_centers)

        colors = {'left': (255, 0, 0), 'center': (0, 0, 255), 'right': (0, 255, 0)}
        for label, center in labels.items():
            if center:
                cv2.circle(roi, center, 7, colors[label], -1)
                cv2.putText(roi, label, (center[0] - 20, center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[label], 2)

        if labels['center']:
            image_center = roi.shape[1] // 2
            error = float(labels['center'][0] - image_center) / image_center
            self.get_logger().info(f'Publishing Error: {error}')
            self.error_publisher.publish(Float32(data=error))
        else:
            self.get_logger().warn('Center line not detected')

        # Publish the processed image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(roi, encoding='bgr8')
            self.debug_image_publisher.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing debug image: {e}')


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