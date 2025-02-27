# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult


#Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')
        
        self.declare_parameter("amplitud",1.0)
        self.declare_parameter("omega",1.0)
        self.declare_parameter("timer_period",0.1)
        self.declare_parameter("signal_type",1)
        
        # Parámetros de la señal globales
        self.amplitude = self.get_parameter("amplitud").value 
        self.omega = self.get_parameter("omega").value 
        self.timer_period = self.get_parameter("timer_period").value 
        self.signal_type = self.get_parameter("signal_type").value 



        #Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        self.add_on_set_parameters_callback(self._parameter_callback)
        
        #Create a messages and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        self.get_logger().info("SetPoint Node Started \U0001F680")

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):
        #Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        # Generate sine wave signal
        if self.signal_type==1:
            self.signal_msg.data=self.amplitude*np.sin(self.omega * elapsed_time) 
        elif self.signal_type==2:
            self.signal_msg.data=self.amplitude*np.cos(self.omega * elapsed_time)
        elif self.signal_type==3:
            self.signal_msg.data=self.amplitude
        
            
        # Publish the signal
        self.signal_publisher.publish(self.signal_msg)

    def _parameter_callback(self,params):
        for param in params:
            if param.name == "amplitud":
                self.amplitude=param.value
            
            elif param.name == "omega":
                self.omega=param.value

            elif param.name == "timer_period":
                self.timer_period=param.value

            elif param.name == "signal_type":
                self.signal_type=param.value

        return SetParametersResult
    

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
