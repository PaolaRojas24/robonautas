import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ManualColorPublisher(Node):
    def __init__(self):
        super().__init__('manual_color_publisher')
        self.publisher_ = self.create_publisher(Int32, '/color', 10)
        self.get_logger().info('🧪 Nodo de prueba para /color iniciado. Escribe 0, 1, 2 o 3 y presiona Enter.')

        # Ejecuta lectura de entrada del usuario en un hilo separado
        import threading
        threading.Thread(target=self.user_input_loop, daemon=True).start()

    def user_input_loop(self):
        while rclpy.ok():
            try:
                user_input = input("Ingresa un número (0=None, 1=Red, 2=Green, 3=Yellow): ")
                value = int(user_input)
                if value in [0, 1, 2, 3]:
                    msg = Int32()
                    msg.data = value
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'📤 Publicado en /color: {value}')
                else:
                    self.get_logger().warn("⚠️ Valor no válido. Usa 0, 1, 2 o 3.")
            except ValueError:
                self.get_logger().warn("⚠️ Entrada inválida. Solo se aceptan números enteros.")

def main(args=None):
    rclpy.init(args=args)
    node = ManualColorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
