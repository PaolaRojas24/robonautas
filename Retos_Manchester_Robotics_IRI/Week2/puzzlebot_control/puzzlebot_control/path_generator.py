import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose 
from std_msgs.msg import Header
from custom_interfaces.msg import TimedPose
import math

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_argos')

        self.pose_pub = self.create_publisher(TimedPose, 'pose', 10)

        self.get_logger().info("Inicializando generación de trayectoria...")

        # Límites definidos por calibración o capacidades del robot
        self.LINEAR_MIN = 0.046800017613798494
        self.LINEAR_MAX = 0.2557205000000001
        self.ANGULAR_MIN = 0.38742048900000015
        self.ANGULAR_MAX = 2.653116706110003

        for i in range(4):
            print(f"\n📍 Punto {i+1}")
            x = self.ask_clamped_coordinate(" Coordenada x: ")
            y = self.ask_clamped_coordinate(" Coordenada y: ")

            #theta_deg = float(input(" Orientación (°): "))
            #theta_rad = math.radians(theta_deg)

            # Construir pose
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.z = 0.0 #math.sin(theta_rad / 2.0)
            pose.orientation.w = 0.0 #math.cos(theta_rad / 2.0)

            mode = ""
            while mode not in ["v", "t"]:
                mode = input(" ¿Este punto se controla por velocidad (v) o tiempo (t)? [v/t]: ").strip().lower()

            if mode == "v":
                linear_speed = self.ask_positive_float(" 🚗 Velocidad lineal (m/s): ")
                angular_speed = self.ask_positive_float(" 🔁 Velocidad angular (rad/s): ")

                # Aplicar límites (clamp)
                mapped_linear = self.clamp(linear_speed, self.LINEAR_MIN, self.LINEAR_MAX)
                mapped_angular = self.clamp(angular_speed, self.ANGULAR_MIN, self.ANGULAR_MAX)

                forward_time = 0.0
                rotate_time = 0.0

                linear_speed = mapped_linear
                angular_speed = mapped_angular

            else:
                distance = math.sqrt(x**2 + y**2)
                angle = abs(math.atan2(y, x))

                forward_time = self.ask_positive_float(" ⏱️ Tiempo para tramo recto (s): ")
                rotate_time = self.ask_positive_float(" ⏱️ Tiempo para giro (s): ")

                # Calcular velocidades estimadas
                estimated_linear = distance / forward_time
                estimated_angular = angle / rotate_time

                # Aplicar límites
                mapped_linear = self.clamp(estimated_linear, self.LINEAR_MIN, self.LINEAR_MAX)
                mapped_angular = self.clamp(estimated_angular, self.ANGULAR_MIN, self.ANGULAR_MAX)

                # Recalcular tiempos si fueron ajustadas
                if mapped_linear != estimated_linear:
                    forward_time = distance / mapped_linear
                if mapped_angular != estimated_angular:
                    rotate_time = angle / mapped_angular

                linear_speed = mapped_linear
                angular_speed = mapped_angular

            # Construir mensaje
            msg = TimedPose()
            msg.pose = pose
            msg.forward_time = forward_time
            msg.rotate_time = rotate_time
            msg.linear_speed = linear_speed
            msg.angular_speed = angular_speed

            self.pose_pub.publish(msg)
            self.get_logger().info(f"🛰️ Publicado punto {i+1}")

    def clamp(self, value, min_val, max_val):
        if value < min_val:
            print(f"⚠️ Valor {value:.4f} menor que el mínimo permitido ({min_val:.4f}). Usando mínimo.")
            return min_val
        elif value > max_val:
            print(f"⚠️ Valor {value:.4f} mayor que el máximo permitido ({max_val:.4f}). Usando máximo.")
            return max_val
        return value

    def ask_positive_float(self, prompt):
        while True:
            try:
                value = float(input(prompt))
                if value > 0:
                    return value
                else:
                    print("❌ El valor debe ser mayor que cero. Intenta de nuevo.")
            except ValueError:
                print("❌ Entrada inválida. Ingresa un número válido.")
    
    def ask_clamped_coordinate(self, prompt):
        while True:
            try:
                value = float(input(prompt))
                if -3.0 <= value <= 3.0:
                    return value
                else:
                    print("❌ La coordenada debe estar entre -3 y 3 metros. Intenta de nuevo.")
            except ValueError:
                print("❌ Entrada inválida. Ingresa un número válido.")


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()