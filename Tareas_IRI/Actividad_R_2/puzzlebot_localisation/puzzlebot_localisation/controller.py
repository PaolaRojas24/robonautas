import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Setpoint objetivo
        self.xg = 1.0  
        self.yg = 0.0

        # Crear un perfil QoS compatible con el publicador /odom (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Suscripción con QoS actualizado
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

    def odom_callback(self, msg):
        # Posiciones actuales
        xr = msg.pose.pose.position.x
        yr = msg.pose.pose.position.y

        # Obtener orientación en ángulo (yaw)
        q = msg.pose.pose.orientation
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        # Cálculo manual del yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        thetar = math.atan2(siny_cosp, cosy_cosp)

        # Calcular errores
        ex = self.xg - xr
        ey = self.yg - yr
        e_theta = math.atan2(ey, ex) - thetar
        e_theta = self.normalize_angle(e_theta)
        e_d = math.sqrt(ex**2 + ey**2)

        # Imprimir por terminal
        print(f"[Controller] error_d: {e_d:.2f}, error_theta: {e_theta:.2f}")

    def normalize_angle(self, angle):
        """Normaliza el ángulo entre -pi y pi"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
