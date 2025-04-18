# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32

#Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        # Retrieve sine wave parameters
        self.amplitude = 2.0
        self.period = 6.28

        #Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'set_point_argos', 10)
        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)
        
        #Create a messages and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        self.get_logger().info("SetPoint Node Started \U0001F680")

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):
        #Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        #sine wave signal

        # square wave
        if (elapsed_time % self.period) < (self.period / 2):
            self.signal_msg.data = self.amplitude
        else:
            self.signal_msg.data = -self.amplitude

        # Publish the signal
        self.signal_publisher.publish(self.signal_msg)

#Main
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPointPublisher()

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