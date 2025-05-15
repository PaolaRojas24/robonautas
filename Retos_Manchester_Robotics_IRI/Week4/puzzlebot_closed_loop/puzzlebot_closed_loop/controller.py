import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from custom_interfaces.msg import PoseK
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class ProportionalIntegralController(Node):
    def __init__(self):
        super().__init__('controller_pi')

        # Objetivo y configuración
        self.xg = 0.0
        self.yg = 0.0
        self.tolerance = 0.115
        self.angle_tolerance = 0.315
        self.goal_reached = False
        self.goal_received = False
        self.confirmed = False

        # Ganancias P desde el mensaje
        self.kp_linear = 0.6
        self.kp_angular = 1.5

        # Ganancias I internas
        self.ki_linear = 0.2
        self.ki_angular = 0.1

        # Acumuladores de error
        self.integral_linear_error = 0.0
        self.integral_angular_error = 0.0

        # Límites de velocidad
        self.LINEAR_MIN = 0.0468
        self.LINEAR_MAX = 0.0557
        self.ANGULAR_MIN = 0.3874
        self.ANGULAR_MAX = 0.6531

        # Estado del semáforo
        self.traffic_light_state = 1  # por defecto: rojo
        self.motion_state = 'STOP'    # STOP, GO, SLOW

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscripciones y publicaciones
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.subscription_goal = self.create_subscription(PoseK, '/goals', self.goal_callback, 10)
        self.subscription_color = self.create_subscription(Int32, '/color', self.color_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.posek_pub = self.create_publisher(PoseK, '/goals', 10)

        self.get_logger().info("🚀 Controlador PI con lógica de semáforo iniciado.")

    def color_callback(self, msg: Int32):
        new_state = msg.data

        if self.motion_state == 'STOP' and new_state == 2:
            self.motion_state = 'GO'
            self.get_logger().info("🟢 Cambiando a estado GO")
        elif self.motion_state == 'GO' and new_state == 3:
            self.motion_state = 'SLOW'
            self.get_logger().info("🟡 Cambiando a estado SLOW")
        elif self.motion_state == 'SLOW' and new_state == 1:
            self.motion_state = 'STOP'
            self.get_logger().info("🔴 Cambiando a estado STOP")

        self.traffic_light_state = new_state

    def goal_callback(self, msg: PoseK):
        if msg.bandera == 1.0:
            return

        self.get_logger().info(f"📨 Nuevo objetivo: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.xg = msg.pose.position.x
        self.yg = msg.pose.position.y
        self.kp_linear = msg.kp_linear
        self.kp_angular = msg.kp_angular

        # Reiniciar errores
        self.integral_linear_error = 0.0
        self.integral_angular_error = 0.0

        self.goal_reached = False
        self.goal_received = True
        self.confirmed = False

    def odom_callback(self, msg):
        if not self.goal_received:
            if not hasattr(self, 'waiting_msg_shown') or not self.waiting_msg_shown:
                self.get_logger().info("🕓 Esperando objetivo...")
                self.waiting_msg_shown = True
            return

        self.waiting_msg_shown = False

        if self.goal_reached and not self.confirmed:
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

        # 🚦 Estado STOP
        if self.motion_state == 'STOP':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info("🚦 Estado: STOP (detenido por semáforo)")
            return

        # ⚠️ Estado SLOW
        elif self.motion_state == 'SLOW':
            slow_factor = 0.3
        else:
            slow_factor = 1.0

        if distance_error < self.tolerance and abs(angular_error) < self.angle_tolerance:
            if not self.goal_reached:
                self.get_logger().info("🎯 Objetivo alcanzado. Deteniendo robot...")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.goal_reached = True
            return

        # Acumuladores de error PI
        self.integral_linear_error += distance_error
        self.integral_angular_error += angular_error

        if abs(angular_error) > 0.1:
            cmd.linear.x = 0.0
            control_angular = (
                self.kp_angular * angular_error +
                self.ki_angular * self.integral_angular_error
            )
            cmd.angular.z = self.clamp(control_angular, self.ANGULAR_MIN, self.ANGULAR_MAX, "angular")
            self.get_logger().info(f"[Giro]: θ_error={angular_error:.2f}, W={cmd.angular.z:.2f}")
        else:
            cmd.angular.z = 0.0
            control_linear = (
                self.kp_linear * distance_error +
                self.ki_linear * self.integral_linear_error
            ) * slow_factor
            cmd.linear.x = self.clamp(control_linear, self.LINEAR_MIN, self.LINEAR_MAX, "lineal")
            self.get_logger().info(f"[Avance-{self.motion_state}]: d_error={distance_error:.2f}, V={cmd.linear.x:.2f}")

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
    node = ProportionalIntegralController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()