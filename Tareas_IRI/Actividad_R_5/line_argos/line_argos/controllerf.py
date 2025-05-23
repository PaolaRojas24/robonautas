import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Ganancia proporcional ajustada a errores normalizados
        self.Kp = 0.5  # Puedes probar entre 0.2 y 1.0 según respuesta del robot
        self.linear_speed = 0.08  # Avance constante

        self.current_error = 0.0

        self.error_subscriber = self.create_subscription(
            Float32,
            '/error',
            self.error_callback,
            10)

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Controller Node (Escala -1 a 1) Iniciado')

    def error_callback(self, msg):
        self.current_error = msg.data
        self.get_logger().info(f'Error recibido: {self.current_error:.3f}')

    def timer_callback(self):
        twist = Twist()

        # Velocidad constante hacia adelante
        twist.linear.x = self.linear_speed

        # Giro proporcional negativo al error
        twist.angular.z = -self.Kp * self.current_error

        self.get_logger().info(f'Comando: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}')
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
