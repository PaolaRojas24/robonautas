# controllerf.py
# Nodo de control principal del Puzzlebot. Implementa un controlador proporcional (P)
# y una máquina de estados para reaccionar a errores de seguimiento de línea,
# semáforos y señales de tránsito detectadas por visión por computadora.

import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
from geometry_msgs.msg import Twist

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # === PARÁMETROS DE CONTROL ===
        self.Kp = 0.5                                # Ganancia proporcional
        self.base_speed = 0.09                       # Velocidad base de avance
        self.angular_speed = 1.5                     # Velocidad angular para giros
        self.giro_angle_base = np.deg2rad(80)        # Giro base de 80°

        # === VARIABLES DE ESTADO ===
        self.current_error = 0.0
        self.motion_state = 'GO'                     # Estado actual según semáforo: STOP, GO, SLOW
        self.active_signal = None                    # Señal de tránsito activa
        self.giro_en_proceso = False

        # === DURACIONES DE MANIOBRAS ===
        self.ahead_duration = 0.5 / self.base_speed
        self.foward_duration = 0.40 / self.base_speed
        self.end_duration = 0.2 / self.base_speed
        self.rotate_duration = self.giro_angle_base / abs(self.angular_speed)

        self.giro_fase = None                        # Fase actual del giro (avance_inicio, giro, avance_final)
        self.giro_fase_start_time = None

        self.persistent_signals = ['give_way', 'roadwork']  # Señales que persisten aunque desaparezcan visualmente

        # === SUSCRIPTORES Y PUBLICADORES ROS ===
        self.error_sub = self.create_subscription(Float32, '/error', self.error_callback, 10)
        self.color_sub = self.create_subscription(Int32, '/color', self.color_callback, 10)
        self.sign_sub = self.create_subscription(Int32, '/sign', self.sign_callback, 10)

        self.reset_pub = self.create_publisher(Bool, '/reset_line', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.confirm_pub = self.create_publisher(Int32, '/confirmacion', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Nodo Controller iniciado')

        # === LÓGICA DE RECUPERACIÓN ===
        self.recovering = False
        self.recover_threshold = 0.05
        self.recover_waiting_signal = False
        self.recover_start_time = None
        self.recover_timeout = 5.0
        self.recover_completed = False

    def error_callback(self, msg):
        # Gestiona el error recibido desde el seguidor de línea
        if msg.data == 2.0:  # Valor especial que indica pérdida de línea
            if not self.recovering and not self.recover_completed:
                self.get_logger().warn("Modo RECOVER activado")
                self.recovering = True
                self.recover_waiting_signal = False
                self.recover_start_time = self.get_clock().now()
                self.active_signal = None
                self.giro_en_proceso = False
                self.giro_fase = None
                self.giro_fase_start_time = None
                self.confirm_pub.publish(Int32(data=1))
        else:
            self.current_error = msg.data
            if self.recovering and not self.recover_waiting_signal and abs(msg.data) <= self.recover_threshold:
                self.get_logger().info("Error corregido, esperando señal para finalizar RECOVER")
                self.recover_waiting_signal = True
                self.recover_start_time = self.get_clock().now()

    def color_callback(self, msg):
        # Cambia el estado de movimiento según el semáforo detectado
        code = msg.data
        if code == 1 and self.motion_state == 'SLOW':
            self.motion_state = 'STOP'
            if self.active_signal != 'stop':
                self.active_signal = None
            self.get_logger().info("Semáforo: STOP")
        elif code == 2:
            self.motion_state = 'GO'
            self.get_logger().info("Semáforo: GO")
        elif code == 3 and self.motion_state == 'GO':
            self.motion_state = 'SLOW'
            self.get_logger().info("Semáforo: SLOW")

    def sign_callback(self, msg):
        # Gestiona la detección de una señal de tránsito
        code = msg.data
        sign_map = {
            4: 'turn_right', 5: 'turn_left', 6: 'ahead',
            7: 'stop', 8: 'give_way', 9: 'roadwork'
        }

        # Si está en modo recover, no acepta nuevas señales hasta finalizar
        if self.recovering and not self.recover_waiting_signal:
            return
        if self.recover_waiting_signal and code in [4, 5, 6]:
            self.get_logger().info(f"Señal detectada tras recover: {sign_map[code]}")
            self.recover_waiting_signal = False
            self.recovering = False
            self.recover_completed = True

        if self.motion_state == 'STOP':
            return

        if code in sign_map:
            nueva_senal = sign_map[code]

            if nueva_senal != self.active_signal:
                self.active_signal = nueva_senal
                self.get_logger().info(f"Nueva señal activa: {self.active_signal}")

                if nueva_senal in ['turn_left', 'turn_right']:
                    # Inicio de giro: dividir en tres fases
                    self.giro_en_proceso = True
                    self.giro_fase = 'avance_inicio'
                    self.giro_fase_start_time = self.get_clock().now()

                    # Ajuste dinámico del ángulo de giro según error actual
                    error = self.current_error if self.current_error is not None else 0.0
                    max_delta_deg = 10
                    correction_deg = max_delta_deg * error if nueva_senal == 'turn_left' else -max_delta_deg * error
                    total_angle = self.giro_angle_base + np.deg2rad(correction_deg)
                    self.rotate_duration = abs(total_angle / self.angular_speed)
                    self.reset_pub.publish(Bool(data=True))

                elif nueva_senal == 'ahead':
                    self.ahead_start_time = self.get_clock().now()
                    self.reset_pub.publish(Bool(data=True))

        elif code == 0 and self.active_signal == 'stop':
            self.get_logger().info("Señal STOP ya no detectada")
            self.active_signal = None
            self.reset_pub.publish(Bool(data=True))

    def timer_callback(self):
        # Lógica principal del controlador ejecutada periódicamente
        twist = Twist()

        # === MODO RECOVER ===
        if self.recovering:
            if self.recover_waiting_signal:
                elapsed = (self.get_clock().now() - self.recover_start_time).nanoseconds * 1e-9
                if elapsed > self.recover_timeout:
                    self.get_logger().warn("Timeout esperando señal tras recover")
                    self.recovering = False
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            twist.linear.x = 0.0
            twist.angular.z = -self.Kp * self.current_error
            twist.angular.z = max(-1.0, min(twist.angular.z, 1.0))
            self.cmd_vel_pub.publish(twist)
            return

        # === SEMÁFORO EN ROJO ===
        if self.motion_state == 'STOP':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        # === SEÑALES DE TRÁNSITO ===
        if self.active_signal == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        elif self.active_signal in ['give_way', 'roadwork']:
            twist.linear.x = self.base_speed * 0.5
            if self.current_error is not None:
                twist.angular.z = -self.Kp * self.current_error
                twist.angular.z = max(-1.0, min(twist.angular.z, 1.0))

        elif self.active_signal == 'ahead':
            if self.motion_state == 'STOP':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                twist.linear.x = self.base_speed if self.motion_state == 'GO' else self.base_speed * 0.3
                twist.angular.z = 0.0
                elapsed = (self.get_clock().now() - self.ahead_start_time).nanoseconds * 1e-9
                if elapsed >= self.ahead_duration:
                    self.active_signal = None
                    self.reset_pub.publish(Bool(data=True))

        elif self.active_signal in ['turn_left', 'turn_right'] and self.giro_en_proceso:
            if self.motion_state == 'STOP':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            elapsed = (self.get_clock().now() - self.giro_fase_start_time).nanoseconds * 1e-9

            if self.giro_fase == 'avance_inicio':
                twist.linear.x = self.base_speed if self.motion_state == 'GO' else self.base_speed * 0.3
                twist.angular.z = 0.0
                if elapsed >= self.foward_duration:
                    self.giro_fase = 'giro'
                    self.giro_fase_start_time = self.get_clock().now()

            elif self.giro_fase == 'giro':
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed if self.active_signal == 'turn_left' else -self.angular_speed
                if elapsed >= self.rotate_duration:
                    self.giro_fase = 'avance_final'
                    self.giro_fase_start_time = self.get_clock().now()

            elif self.giro_fase == 'avance_final':
                twist.linear.x = self.base_speed if self.motion_state == 'GO' else self.base_speed * 0.3
                twist.angular.z = 0.0
                if elapsed >= self.end_duration:
                    self.giro_en_proceso = False
                    self.active_signal = None
                    self.giro_fase = None
                    self.giro_fase_start_time = None
                    self.reset_pub.publish(Bool(data=True))

        # === CONTROL NORMAL ===
        elif self.current_error is not None:
            twist.linear.x = self.base_speed if self.motion_state == 'GO' else self.base_speed * 0.3
            twist.angular.z = -self.Kp * self.current_error
            twist.angular.z = max(-1.0, min(twist.angular.z, 1.0))

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Reiniciar flag recover_completed si el error ya es bajo
        if not self.recovering and abs(self.current_error) <= self.recover_threshold:
            self.recover_completed = False

        self.cmd_vel_pub.publish(twist)


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

