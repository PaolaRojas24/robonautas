import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.Kp = 0.5  # Ganancia proporcional
        #self.base_speed = 0.04  # Pista completa_la rara 
        self.base_speed = 0.09  # Curvas pronunciadas
        #self.base_speed = 0.11 #Giro en 90 grados

        self.current_error = None
        self.traffic_light_state = 1  # Por defecto en 1
        self.motion_state = 'STOP'    # 'STOP', 'GO', 'SLOW'

        # Subscripciones
        self.error_subscriber = self.create_subscription(Float32, '/error', self.error_callback, 10)
        self.color_subscriber = self.create_subscription(Int32, '/color', self.color_callback, 10)

        # Publicador
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer para envío periódico
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('🚀 Nodo Controller iniciado con lógica de semáforo')

    def error_callback(self, msg):
        self.current_error = msg.data

    def color_callback(self, msg):
        new_state = msg.data

        if new_state == 2 and self.motion_state == 'STOP':
            self.motion_state = 'GO'
            self.get_logger().info("🟢 Semáforo: GO")
        elif new_state == 3 and self.motion_state == 'GO':
            self.motion_state = 'SLOW'
            self.get_logger().info("🟡 Semáforo: SLOW")
        elif new_state == 1 and self.motion_state == 'SLOW':
            self.motion_state = 'STOP'
            self.get_logger().info("🔴 Semáforo: STOP")

        self.traffic_light_state = new_state

    def timer_callback(self):
        twist = Twist()

        if self.motion_state == 'STOP':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info("🚦 Estado: STOP (detenido)")
            return

        if self.current_error is None:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn('⚠️ No hay error recibido')
        else:
            # Determinar velocidad lineal según estado
            if self.motion_state == 'SLOW':
                linear_speed = self.base_speed * 0.3
            else: 
                linear_speed = self.base_speed

            twist.linear.x = linear_speed
            twist.angular.z = -self.Kp * self.current_error

            # Limitar angular
            max_angular_speed = 1.0
            twist.angular.z = max(-max_angular_speed, min(twist.angular.z, max_angular_speed))

            self.get_logger().info(f'🏁 Cmd: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f} [{self.motion_state}]')

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
