import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge
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
        self.get_logger().info('Line Tracker Node Initialized')

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

        image_center_x = roi.shape[1] // 2
        closest_center = None
        min_distance = float('inf')

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    distance = abs(cx - image_center_x)
                    if distance < min_distance:
                        min_distance = distance
                        closest_center = (cx, cy)
                    cv2.drawContours(roi, [cnt], -1, (0, 255, 0), 2)

        if closest_center:
            cx, cy = closest_center
            cv2.circle(roi, (cx, cy), 7, (0, 0, 255), -1)
            cv2.putText(roi, 'center', (cx - 20, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            error = float(cx - image_center_x) / image_center_x
            self.get_logger().info(f'Publishing Error: {error}')
            self.error_publisher.publish(Float32(data=error))
        else:
            self.get_logger().warn('No center line detected')

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