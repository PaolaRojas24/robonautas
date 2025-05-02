import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math


class ProportionalController(Node):
    def __init__(self):
        super().__init__('controller_p')

        # Lista de puntos del cuadrado
        self.targets = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0)
        ]
        self.target_index = 0
        self.tolerance = 0.115
        self.angle_tolerance = 0.1

        # Ganancias proporcionales
        self.kp_linear = 0.6
        self.kp_angular = 1.5

        # Límites para velocidades
        self.LINEAR_MIN = 0.0468
        self.LINEAR_MAX = 0.2557
        self.ANGULAR_MIN = 0.3874
        self.ANGULAR_MAX = 0.6531

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Suscripción a la odometría
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        # Publicador de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("🟩 Controlador cuadrado iniciado.")

    def odom_callback(self, msg):
        if self.target_index >= len(self.targets):
            return  # Ya completó todos los objetivos

        xr = msg.pose.pose.position.x
        yr = msg.pose.pose.position.y

        # Conversión de cuaternión a ángulo theta
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta_r = math.atan2(siny_cosp, cosy_cosp)

        # Objetivo actual
        xg, yg = self.targets[self.target_index]
        dx = xg - xr
        dy = yg - yr
        distance_error = math.sqrt(dx ** 2 + dy ** 2)
        angle_to_goal = math.atan2(dy, dx)
        angular_error = self.normalize_angle(angle_to_goal - theta_r)

        cmd = Twist()

        if distance_error < self.tolerance and abs(angular_error) < self.angle_tolerance:
            self.get_logger().info(f"🎯 Objetivo {self.target_index+1} alcanzado.")
            self.target_index += 1
            if self.target_index >= len(self.targets):
                self.get_logger().info("🏁 Ruta cuadrada completada.")
                self.cmd_pub.publish(Twist())  # Detener el robot
                return
            return  # Esperar al siguiente ciclo

        # Control angular prioritario
        if abs(angular_error) > self.angle_tolerance:
            cmd.linear.x = 0.0
            raw_angular = self.kp_angular * angular_error
            cmd.angular.z = self.clamp(raw_angular, self.ANGULAR_MIN, self.ANGULAR_MAX, "angular")
        else:
            cmd.angular.z = 0.0
            raw_linear = self.kp_linear * distance_error
            cmd.linear.x = self.clamp(raw_linear, self.LINEAR_MIN, self.LINEAR_MAX, "lineal")

        self.cmd_pub.publish(cmd)

    def clamp(self, value, min_val, max_val, tipo):
        if abs(value) < min_val:
            self.get_logger().warn(f"⚠️ Velocidad {tipo} ({value:.4f}) menor que el mínimo ({min_val:.4f}). Usando mínimo.")
            return math.copysign(min_val, value)
        elif abs(value) > max_val:
            self.get_logger().warn(f"⚠️ Velocidad {tipo} ({value:.4f}) mayor que el máximo ({max_val:.4f}). Usando máximo.")
            return math.copysign(max_val, value)
        return value

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = ProportionalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
