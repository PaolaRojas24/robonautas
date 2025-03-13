# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

#Class Definition
class InputPublisher(Node):
    def __init__(self):
        super().__init__('input_node')

        # Retrieve wave parameters
        self.declare_parameter('amplitude', 60.0)
        self.declare_parameter('period', 1.0)
        self.declare_parameter('type',1)
        self.declare_parameter('observador', 0)
        self.declare_parameter('frecuencia', 0.1)

        self.amplitude= self.get_parameter('amplitude').value
        self.period = self.get_parameter('period').value
        self.type = self.get_parameter('type').value
        self.observador = self.get_parameter('observador').value
        self.frecuencia = self.get_parameter('frecuencia').value

        #Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'set_point_argos', 10)
        timer_period = self.frecuencia
        self.timer = self.create_timer(timer_period, self.timer_cb)

        self.motor_output_subscriber = self.create_subscription(Float32, 'motor_output_argos', self.motor_output_callback, 10)
        
        #Create a messages and variables to be used
        self.start_time = self.get_clock().now()
        
        #Set the messages
        self.signal_msg = Float32()
        self.motor_output_msg = Float32()

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("Input Node Started 🌊🚢🌊")

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):
        #Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9

        #Selection of signal
        if self.type == 1:
            signal= self.amplitude * np.sin(self.period * elapsed_time) #sinusoidal
        elif self.type == 2:
            signal= self.amplitude * np.sign(np.sin(self.period * elapsed_time)) #square wave
        elif self.type == 3:
            signal= self.amplitude * (elapsed_time >= 0)

        # Publish the signal
        self.signal_msg.data= signal
        self.signal_publisher.publish(self.signal_msg)
    def motor_output_callback(self, msg):
        self.motor_output_msg = msg
        if self.observador == 1:
            self.get_logger().info(f"Motor Output: {self.motor_output_msg.data}")
            
    def parameters_callback (self, params):
        for param in params:
            if param.name == "amplitude":
                if (param.value < 0.0):
                    self.get_logger().warn(" \u274C Invalid amplitude! It cannot be negative. \u274C ")
                    return SetParametersResult(successful=False, reason="Amplitude cannot be negative")
                else:
                    self.amplitude = param.value # Update internal variable
                    self.get_logger().info(f"Amplitude update to {self.amplitude}")
            if param.name == "period":
                if (param.value < 0.0):
                    self.get_logger().warn("\u274C Invalid period! It cannot be negative.\u274C ")
                    return SetParametersResult(successful=False, reason="Period cannot be negative")
                else:
                    self.period = param.value # Update internal variable
                    self.get_logger().info(f"Period update to {self.period}")
            if param.name == "frecuencia":
                if param.value <= 0.0:
                    self.get_logger().warn("\u274C Invalid frequency! It must be positive. \u274C")
                    return SetParametersResult(successful=False, reason="Frequency must be positive")
                else:
                    self.frecuencia = param.value
                    self.period = 1.0 / self.frecuencia  # Update period based on frequency
                    self.get_logger().info(f"Frequency updated to {self.frecuencia}")

            if param.name == "type":
                if param.value < 1 or param.value > 3:
                    self.get_logger().warn("\u274C  Invalid type! Must be 1 (sinusoidal), 2 (square wave), or 3 (step). \u274C ")
                    return SetParametersResult(successful=False, reason="type must be 1, 2, or 3")
                else:
                    self.type = param.value  # Update internal variable
                    self.get_logger().info(f"Type updated to {self.type}")
            if param.name == "observador":
                self.observador = param.value  # Update internal variable
                self.get_logger().info(f"Observador updated to {self.observador}")

        return SetParametersResult(successful=True)
#Main
def main(args=None):
    rclpy.init(args=args)

    set_point = InputPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()