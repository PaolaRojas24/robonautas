import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

class Ctrl(Node):
    def __init__(self):
        super().__init__('signal_proce')
        #DEclaramos los parámetros del controlador PID7

        self.declare_parameter('gain_Kp',0.33)
        self.declare_parameter('gain_Ki',0.8)
        self.declare_parameter('gain_Kd',0.035)
        self.declare_parameter('sample_time',0.1)

        self.gain_Kp = self.get_parameter('gain_Kp').value
        self.gain_Ki = self.get_parameter('gain_Ki').value
        self.gain_Kd = self.get_parameter('gain_Kd').value
        self.sample_time = self.get_parameter('sample_time').value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.subscription_1 = self.create_subscription(
            Float32, 'set_point_argos', self.sp_callback, 10)
        self.subscription_2 = self.create_subscription(
            Float32, 'motor_output_y_argos', self.signal_callback, 10)
        
        self.publisher = self.create_publisher(Float32, 'motor_input_u_argos', 10)

        self.motor_output_msg = Float32()

        # Timer
        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        self.set_point = 0.0
        self.motor_output= 0.0
        self.error = 0.0
        self.prev = 0.0
        self.inte= 0.0


    def sp_callback(self, msg):
        self.set_point = msg.data

    # Callback para el canal 'motor_output_y_argos'
    def signal_callback(self, msg):
        self.motor_output = msg.data

    def timer_cb(self):
        #Nos ayuda a calcular el error
        self.error= self.set_point - self.motor_output
        #Término integral con limitación
        self.inte += self.error * self.sample_time
        self.inte = np.clip(self.inte, -10, 10)


        # Términos del PID
        #Proporcional, Integral y DErivativo
        P_term = self.gain_Kp * self.error
        I_term = self.gain_Ki * self.inte
        # Filtro derivativo para reducir ruido
        D_term = self.gain_Kd * ((self.error - self.prev) / self.sample_time)


        #salida
        #Proporcional, Integral y DErivativo
        motor_input_u= P_term + I_term - D_term 


        self.prev= self.error

        self.motor_output_msg.data = motor_input_u
        self.publisher.publish(self.motor_output_msg)

        self.get_logger().info(f'Control: {self.motor_output_msg.data}')

    def parameters_callback(self, params):
        for param in params:
            if param.name == "gain_Kp":
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="gain_Kp cannot be negative")
                self.gain_Kp = param.value
                self.get_logger().info(f"gain_Kp updated to {self.gain_Kp}")

            if param.name == "gain_Ki":
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="gain_Ki cannot be negative")
                self.gain_Ki = param.value
                self.get_logger().info(f"gain_Ki updated to {self.gain_Ki}")

            if param.name == "gain_Kd":
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="gain_Kd cannot be negative")
                self.gain_Kd = param.value
                self.get_logger().info(f"gain_Kd updated to {self.gain_Kd}")

            if param.name == "sample_time":
                if param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="sample_time must be positive")
                self.sample_time = param.value
                self.get_logger().info(f"sample_time updated to {self.sample_time}")
                
                # Reiniciar el temporizador con el nuevo sample_time
                self.timer.cancel()
                self.timer = self.create_timer(self.sample_time, self.timer_cb)

        return SetParametersResult(successful=True)

# Main
def main(args=None):
    rclpy.init(args=args)

    controll = Ctrl()

    try:
        rclpy.spin(controll)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        controll.destroy_node()

if __name__ == '__main__':
    main()