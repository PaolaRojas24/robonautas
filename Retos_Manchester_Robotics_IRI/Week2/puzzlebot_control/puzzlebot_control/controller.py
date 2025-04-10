import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_interfaces.msg import TimedPose
import math

class ControllerFSM(Node):
    def __init__(self):
        super().__init__('controller_argos')

        # Estados
        self.IDLE = 0
        self.ROTATE_TO_GOAL = 1
        self.FORWARD = 2

        self.state = self.IDLE
        self.current_pose = None
        self.state_start_time = None

        self.subscription = self.create_subscription(TimedPose, 'pose', self.pose_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🚀 Nodo controller_argos iniciado.")

    def pose_callback(self, msg):
        if self.state == self.IDLE:
            self.current_pose = msg
            self.state = self.ROTATE_TO_GOAL
            self.state_start_time = self.get_clock().now()
            self.get_logger().info("📬 Recibido punto objetivo. Iniciando rotación.")

    def control_loop(self):
        if self.state == self.IDLE or self.current_pose is None:
            self.stop()  # Asegura que el robot se detenga mientras espera
            return

        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9

        pose = self.current_pose.pose
        x = pose.position.x
        y = pose.position.y

        distance = math.sqrt(x**2 + y**2)
        angle_to_goal = math.atan2(y, x)

        linear_speed = self.current_pose.linear_speed
        angular_speed = self.current_pose.angular_speed
        forward_time = self.current_pose.forward_time
        rotate_time = self.current_pose.rotate_time

        twist = Twist()

        if self.state == self.ROTATE_TO_GOAL:
            duration = rotate_time if rotate_time > 0 else abs(angle_to_goal) / angular_speed
            actual_angular = angle_to_goal / duration if rotate_time > 0 else angular_speed
            twist.angular.z = actual_angular

            if elapsed >= duration:
                self.stop()
                self.state = self.FORWARD
                self.state_start_time = now
                self.get_logger().info("✅ Giro hacia el objetivo completado.")

        elif self.state == self.FORWARD:
            duration = forward_time if forward_time > 0 else distance / linear_speed
            actual_linear = distance / duration if forward_time > 0 else linear_speed
            twist.linear.x = actual_linear

            if elapsed >= duration:
                self.stop()
                self.state = self.IDLE
                self.current_pose = None
                self.get_logger().info("✅ Movimiento recto completado. Esperando siguiente punto...")

        self.cmd_vel_pub.publish(twist)

    def stop(self):
        # Publica un mensaje de velocidad cero
        self.cmd_vel_pub.publish(Twist())

    def extract_yaw(self, orientation):
        z = orientation.z
        w = orientation.w
        return 2 * math.atan2(z, w)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Nodo detenido por usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()