import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Float32

#Class Definition
class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_gen_node')
        self.publisher1 = self.create_publisher(Float32, 'signal_argos', 10)
        self.publisher2 = self.create_publisher(Float32, 'time_argos', 10)
        timer_period = 0.1 #10Hz
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.i = 0.0
        
    #Timer Callback
    def timer_cb(self):
        i_msg = Float32()
        i_msg.data = self.i
        self.publisher2.publish(i_msg)

        w_msg = Float32()
        w_msg.data = np.sin(self.i)
        self.publisher1.publish(w_msg)

        self.get_logger().info('Signal: "%f"' % w_msg.data)
        self.get_logger().info('Time: "%f"' % i_msg.data)
        self.i += 0.1

#Main
def main(args=None):
    rclpy.init(args=args)

    signal_generator = SignalGenerator()

    try:
        rclpy.spin(signal_generator)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        signal_generator.destroy_node()


#Execute Node
if __name__ == '__main__':
    main()