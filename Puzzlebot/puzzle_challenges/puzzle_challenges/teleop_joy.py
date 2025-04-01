import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rcl_interfaces.msg import SetParametersResult
import sys
import termios
import tty
import threading

class teleop_joy_Publisher(Node):
    def __init__(self):
        super().__init__('teleop_joy')
        
        self.declare_parameter("linear_x", 0.0)
        self.declare_parameter("angular_z", 0.0)

        self.twist_msg = Twist()
        self.twist_msg.linear.x = self.get_parameter("linear_x").value
        self.twist_msg.angular.z = self.get_parameter("angular_z").value

        self.signal_publisher_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.add_on_set_parameters_callback(self._parameter_callback)
        self.get_logger().info("SetPoint Node Started Xbox Controller Support 🚗")

    def limit_speed_v(self, value):
        return max(-0.1, min(0.1, value))
    def limit_speed_w(self, value):
        return max(-0.3, min(0.3, value))

    
    def joy_callback(self, msg):
        left_y = msg.axes[1]
        right_x = msg.axes[3]
        
        max_speed = 0.1
        max_turn_speed = 0.3
        
        self.twist_msg.linear.x = self.limit_speed_v(left_y * max_speed)
        self.twist_msg.angular.z = self.limit_speed_w(right_x * max_turn_speed )
              
    def timer_cb(self):
        self.signal_publisher_vel.publish(self.twist_msg)
    
    def _parameter_callback(self, params):
        for param in params:
            if param.name == "linear_x":
                self.twist_msg.linear.x = self.limit_speed_v(param.value)
            elif param.name == "angular_z":
                self.twist_msg.angular.z = self.limit_speed_w(param.value)     
        return SetParametersResult()
    
def main(args=None):
    rclpy.init(args=args)
    teleop_joy = teleop_joy_Publisher() 
    try:
        rclpy.spin(teleop_joy)
        
    except KeyboardInterrupt:
        pass
    finally:
        teleop_joy.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
