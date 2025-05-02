import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from custom_interfaces.msg import PoseK
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math


class ProportionalController(Node):
    def __init__(self):
        super().__init__('controller_p')

        # Variables objetivo y configuración
        self.xg = 0.0
        self.yg = 0.0
        self.tolerance = 0.115
        self.angle_tolerance = 0.1  # ⬅️ Más estricta para mayor precisión angular
        self.goal_reached = False
        self.goal_received = False
        self.confirmed = False

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

        # Suscripciones
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        self.subscription_goal = self.create_subscription(
            PoseK,
            '/goals',
            self.goal_callback,
            10
        )

        # Publicadores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.posek_pub = self.create_publisher(PoseK, '/goals', 10)

        self.get_logger().info("🚀 Controlador P iniciado correctamente.")

    def goal_callback(self, msg: PoseK):
        # Ignorar si el mensaje es una confirmación
        if msg.bandera == 1.0:
            return

        self.get_logger().info(f"📨 Nuevo objetivo: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.xg = msg.pose.position.x
        self.yg = msg.pose.position.y
        self.kp_linear = msg.kp_linear
        self.kp_angular = msg.kp_angular
        self.goal_reached = False
        self.goal_received = True
        self.confirmed = False


    def odom_callback(self, msg):
        if not self.goal_received:
            if not hasattr(self, 'waiting_msg_shown') or not self.waiting_msg_shown:
                self.get_logger().info("🕓 Esperando objetivo...")
                self.waiting_msg_shown = True
            return

        # Resetea la variable para futuros objetivos
        self.waiting_msg_shown = False

        if self.goal_reached and not self.confirmed:
            # Publica confirmación al path_generator
            confirm_msg = PoseK()
            confirm_msg.pose.position.x = self.xg
            confirm_msg.pose.position.y = self.yg
            confirm_msg.kp_linear = self.kp_linear
            confirm_msg.kp_angular = self.kp_angular
            confirm_msg.bandera = 1.0
            self.posek_pub.publish(confirm_msg)
            self.confirmed = True
            self.get_logger().info("✅ Confirmación de objetivo enviada.")
            return

        xr = msg.pose.pose.position.x
        yr = msg.pose.pose.position.y

        # Conversión de cuaternión a ángulo theta
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta_r = math.atan2(siny_cosp, cosy_cosp)

        dx = self.xg - xr
        dy = self.yg - yr
        distance_error = math.sqrt(dx ** 2 + dy ** 2)
        angle_to_goal = math.atan2(dy, dx)
        angular_error = self.normalize_angle(angle_to_goal - theta_r)

        cmd = Twist()

        if distance_error < self.tolerance and abs(angular_error) < self.angle_tolerance:
            if not self.goal_reached:
                self.get_logger().info("🎯 Objetivo alcanzado. Deteniendo robot...")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.goal_reached = True
            return

        if abs(angular_error) > 0.1:
            cmd.linear.x = 0.0
            raw_angular = self.kp_angular * angular_error
            cmd.angular.z = self.clamp(raw_angular, self.ANGULAR_MIN, self.ANGULAR_MAX, "angular")
            self.get_logger().info(f"Giro]: θ_error={angular_error:.2f}, W={cmd.angular.z:.2f}")
        else:
            cmd.angular.z = 0.0
            raw_linear = self.kp_linear * distance_error
            cmd.linear.x = self.clamp(raw_linear, self.LINEAR_MIN, self.LINEAR_MAX, "lineal")
            self.get_logger().info(f"[Avance]: d_error={distance_error:.2f}, V={cmd.linear.x:.2f}")

        self.cmd_pub.publish(cmd)


    def clamp(self, value, min_val, max_val, tipo):
        if abs(value) < min_val:
            self.get_logger().warn(f"⚠️ Velocidad {tipo} ({value:.4f}) menor que el mínimo permitido ({min_val:.4f}). Usando mínimo.")
            return math.copysign(min_val, value)
        elif abs(value) > max_val:
            self.get_logger().warn(f"⚠️ Velocidad {tipo} ({value:.4f}) mayor que el máximo permitido ({max_val:.4f}). Usando máximo.")
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
