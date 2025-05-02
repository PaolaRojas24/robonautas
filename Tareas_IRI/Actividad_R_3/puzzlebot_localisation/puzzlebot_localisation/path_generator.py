import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PoseK
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time


class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        # Lista de objetivos y estado
        self.goals = []
        self.index = 0
        self.goal_confirmed = True
        self.first_sent = False

        # Publicador
        self.publisher_ = self.create_publisher(PoseK, '/goals', 10)

        # Subscripción para confirmar desde el controlador
        self.subscription = self.create_subscription(
            PoseK,
            '/goals',
            self.confirm_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.get_logger().info("🧭 Generador de trayectorias iniciado. Esperando entrada del usuario...")

        # Entrada de usuario
        self.read_goals_from_user()

        # Envío del primer objetivo
        self.send_next_goal()

    def read_goals_from_user(self):
        try:
            num_goals = int(input("¿Cuántos puntos deseas enviar? "))
            for i in range(num_goals):
                print(f"\n🔸 Punto {i+1}")
                x = float(input("  ↪ Coordenada X: "))
                y = float(input("  ↪ Coordenada Y: "))
                kp_linear = float(input("  ↪ Kp Lineal (ej. 0.6): "))
                kp_angular = float(input("  ↪ Kp Angular (ej. 1.5): "))

                self.goals.append({
                    'x': x,
                    'y': y,
                    'kp_linear': kp_linear,
                    'kp_angular': kp_angular
                })

        except Exception as e:
            self.get_logger().error(f"❌ Error en la entrada: {e}")
            rclpy.shutdown()

    def send_next_goal(self):
        if self.index < len(self.goals):
            goal = self.goals[self.index]
            msg = PoseK()
            msg.pose.position.x = goal['x']
            msg.pose.position.y = goal['y']
            msg.kp_linear = goal['kp_linear']
            msg.kp_angular = goal['kp_angular']
            msg.bandera = 0.0
            self.publisher_.publish(msg)

            if not self.first_sent:
                self.get_logger().info(f"🚀 Primera posición enviada: ({goal['x']:.2f}, {goal['y']:.2f})")
                self.first_sent = True
            else:
                self.get_logger().info(f"➡️ Nueva posición enviada: ({goal['x']:.2f}, {goal['y']:.2f})")

            self.goal_confirmed = False

    def confirm_callback(self, msg: PoseK):
        if self.index < len(self.goals):
            current_goal = self.goals[self.index]

            if (msg.bandera == 1.0 and
                abs(msg.pose.position.x - current_goal['x']) < 0.01 and
                abs(msg.pose.position.y - current_goal['y']) < 0.01):

                self.get_logger().info(f"✅ Confirmación recibida del punto {self.index+1}")
                self.index += 1
                self.goal_confirmed = True

                if self.index < len(self.goals):
                    time.sleep(1)  # Pausa antes de enviar el siguiente objetivo
                    self.send_next_goal()
                else:
                    self.get_logger().info("🎉 Todos los puntos han sido enviados y confirmados.")


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
