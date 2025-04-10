# controller_square_argos.py
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
import numpy as np
from geometry_msgs.msg import Twist # type: ignore

class SquareCtrl(Node):
    def __init__(self):
        super().__init__('controller_square_argos')

        self.wait_for_ros_time()

        def clamp(value, min_val, max_val):
            if value < min_val:
                print(f"⚠️ Valor {value:.4f} menor que el mínimo permitido ({min_val:.4f}). Usando mínimo.")
                return min_val
            elif value > max_val:
                print(f"⚠️ Valor {value:.4f} mayor que el máximo permitido ({max_val:.4f}). Usando máximo.")
                return max_val
            return value

        def ask_positive_float(prompt):
            while True:
                try:
                    value = float(input(prompt))
                    if value > 0:
                        return value
                    else:
                        print("❌ El valor debe ser mayor que cero. Intenta de nuevo.")
                except ValueError:
                    print("❌ Entrada inválida. Ingresa un número válido.")

        # === Entrada con validación y mapeo ===
        LINEAR_MIN = 0.046800017613798494
        LINEAR_MAX = 0.2557205000000001
        ANGULAR_MIN = 0.38742048900000015
        ANGULAR_MAX = 2.653116706110003

        mode = ""
        while mode not in ["v", "t"]:
            mode = input("¿Quieres controlar por velocidad (v) o tiempo (t)? [v/t]: ").strip().lower()

        if mode == 't':
            forward = ask_positive_float("⏱️ Ingresa el tiempo para cada tramo recto (segundos): ")
            rotate = ask_positive_float("⏱️ Ingresa el tiempo para cada giro de 90° (segundos): ")

            linear = 2.0 / forward
            angular = np.deg2rad(80) / rotate

            mapped_linear = clamp(linear, LINEAR_MIN, LINEAR_MAX)
            mapped_angular = clamp(angular, ANGULAR_MIN, ANGULAR_MAX)

            if mapped_linear != linear:
                forward = 2.0 / mapped_linear
            if mapped_angular != angular:
                rotate = np.deg2rad(80) / mapped_angular

            self.linear_speed = mapped_linear
            self.angular_speed = mapped_angular
            self.forward_time = forward
            self.rotate_time = rotate

            self.get_logger().info("Modo: autoajuste por tiempo")

        else:
            linear = ask_positive_float("🚗 Ingresa la velocidad lineal (m/s) para los tramos rectos: ")
            angular = ask_positive_float("🔁 Ingresa la velocidad angular (rad/s) para los giros: ")

            self.linear_speed = clamp(linear, LINEAR_MIN, LINEAR_MAX)
            self.angular_speed = clamp(angular, ANGULAR_MIN, ANGULAR_MAX)

            self.forward_time = 2.0 / self.linear_speed
            self.rotate_time = np.deg2rad(80) / self.angular_speed

            self.get_logger().info("Modo: velocidad fija")

        # Mostrar configuración
        self.get_logger().info(f"Velocidad lineal: {self.linear_speed:.2f} m/s")
        self.get_logger().info(f"Velocidad angular: {self.angular_speed:.2f} rad/s")
        self.get_logger().info(f"Duración de tramo recto: {self.forward_time:.2f}s")
        self.get_logger().info(f"Duración del giro: {self.rotate_time:.2f}s")

        input("✅ Configuración completa. Presiona Enter para comenzar la ejecución...")

        # Publicador
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Inicialización de estados
        self.state = 0
        self.state_start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.state_start_time).nanoseconds * 1e-9

        self.get_logger().info(f"Start: {self.state_start_time.nanoseconds * 1e-9}, NOW: {now.nanoseconds * 1e-9:.2f}s")
        self.get_logger().info(f"State: {self.state}, Elapsed: {elapsed_time:.2f}s")

        cmd = Twist()

        if self.state == 0:
            # Move forward
            cmd.linear.x = self.linear_speed
            self.get_logger().info('Moving forward...')
            if elapsed_time >= self.forward_time:
                self.state = 1
                self.state_start_time = now
                self.get_logger().info('Finished moving forward. Starting rotation...')

        elif self.state == 1:
            # Rotate 90 degrees
            cmd.angular.z = self.angular_speed
            self.get_logger().info('Rotating 90 degrees...')
            if elapsed_time >= self.rotate_time:
                self.state = 2
                self.state_start_time = now
                self.get_logger().info('Finished rotation. Moving forward...')

        elif self.state == 2:
            # Move forward
            cmd.linear.x = self.linear_speed
            self.get_logger().info('Moving forward...')
            if elapsed_time >= self.forward_time:
                self.state = 3
                self.state_start_time = now
                self.get_logger().info('Finished moving forward. Starting rotation...')

        elif self.state == 3:
            # Rotate 90 degrees
            cmd.angular.z = self.angular_speed
            self.get_logger().info('Rotating 90 degrees...')
            if elapsed_time >= self.rotate_time:
                self.state = 4
                self.state_start_time = now
                self.get_logger().info('Finished rotation. Moving forward...')
        
        elif self.state == 4:
            # Move forward
            cmd.linear.x = self.linear_speed
            self.get_logger().info('Moving forward...')
            if elapsed_time >= self.forward_time:
                self.state = 5
                self.state_start_time = now
                self.get_logger().info('Finished moving forward. Starting rotation...')

        elif self.state == 5:
            # Rotate 90 degrees
            cmd.angular.z = self.angular_speed
            self.get_logger().info('Rotating 90 degrees...')
            if elapsed_time >= self.rotate_time:
                self.state = 6
                self.state_start_time = now
                self.get_logger().info('Finished rotation. Moving forward...')

        elif self.state == 6:
            # Move forward
            cmd.linear.x = self.linear_speed
            self.get_logger().info('Moving forward...')
            if elapsed_time >= self.forward_time:
                self.state = 7
                self.state_start_time = now
                self.get_logger().info('Finished moving forward. Starting rotation...')

        elif self.state == 7:
            # Rotate 90 degrees
            cmd.angular.z = self.angular_speed
            self.get_logger().info('Rotating 90 degrees...')
            if elapsed_time >= self.rotate_time:
                self.state = 8
                self.state_start_time = now
                self.get_logger().info('Finished rotation...')
        
        elif self.state == 8:
            # Stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Stopped.')
            # Optionally: cancel the timer after stopping
            self.timer.cancel()

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)

        self.cmd_vel_pub.publish(cmd)

    def wait_for_ros_time(self):
        self.get_logger().info('Esperando a que el tiempo de ROS esté activo...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Tiempo de ROS activo.')

def main(args=None):
    rclpy.init(args=args)
    node = SquareCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()