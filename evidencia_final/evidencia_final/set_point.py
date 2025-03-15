import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from rcl_interfaces.msg import SetParametersResult
import sys
import termios
import tty
import threading

class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point')
        
        self.declare_parameter("vel_d", 0.0)
        self.declare_parameter("vel_i", 0.0)
        self.declare_parameter("timer_period", 0.1)

        self.vel_d = self.get_parameter("vel_d").value 
        self.vel_i = self.get_parameter("vel_i").value     
        self.timer_period = self.get_parameter("timer_period").value 

        self.signal_publisher_d = self.create_publisher(Float32, 'set_point_d', 10)
        self.signal_publisher_i = self.create_publisher(Float32, 'set_point_i', 10)
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.timer = self.create_timer(self.timer_period, self.timer_cb)
        self.add_on_set_parameters_callback(self._parameter_callback)
        
        self.vel_d_msg = Float32()
        self.vel_i_msg = Float32()
        self.start_time = self.get_clock().now()
        
        self.running = True
        self.thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.thread.start()

        self.get_logger().info("SetPoint Node Started with Keyboard & Xbox Controller Support 🚗")
        self.get_logger().info("KeyBoard= ")
        self.get_logger().info("Q= Llanta Izquierda +0.1   A= Llanta Izquierda -0.1")
        self.get_logger().info("C= Detener Programa")

    def limit_speed(self, value):
        return max(-0.4, min(0.4, value))

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def keyboard_listener(self):
        while self.running:
            key = self.get_key()
            if key == 'c':
                self.vel_d = 0.0
                self.vel_i = 0.0
                self.timer_cb()

                self.get_logger().info("Cerrando el programa...")
                self.running = False
                rclpy.shutdown()
                sys.exit(0)
            self.process_key_input(key)
    
    def process_key_input(self, key):
        if key == 'r':
            self.vel_d = self.limit_speed(self.vel_d + 0.1)
            self.vel_i = self.limit_speed(self.vel_i + 0.1)
        elif key == 'q':
            self.vel_i = self.limit_speed(self.vel_i + 0.1)
        elif key == 'w':
            self.vel_d = self.limit_speed(self.vel_d + 0.1)
        elif key == 'a':
            self.vel_i = self.limit_speed(self.vel_i - 0.1)
        elif key == 's':
            self.vel_d = self.limit_speed(self.vel_d - 0.1)
        elif key == 'f':
            self.vel_d = self.limit_speed(self.vel_d - 0.1)
            self.vel_i = self.limit_speed(self.vel_i - 0.1)
        elif key == '0':
            self.vel_d = 0.0
            self.vel_i = 0.0
        self.get_logger().info(f"Velocidad Izquierda: {self.vel_i:.1f}, Velocidad Derecha: {self.vel_d:.1f}")
    
    def joy_callback(self, msg):
        left_y = msg.axes[1]
        right_x = msg.axes[3]
        
        max_speed = 0.4
        max_turn_speed = 0.2
        
        self.vel_i = self.limit_speed(left_y * max_speed - right_x * max_turn_speed)
        self.vel_d = self.limit_speed(left_y * max_speed + right_x * max_turn_speed)
        
        if len(msg.buttons) > 0 and msg.buttons[0]:
            self.vel_i = self.limit_speed(self.vel_i - 0.1)
        if len(msg.buttons) > 1 and msg.buttons[1]:
            self.vel_d = self.limit_speed(self.vel_d - 0.1)
        if len(msg.buttons) > 2 and msg.buttons[2]:
            self.vel_i = self.limit_speed(self.vel_i + 0.1)
        if len(msg.buttons) > 3 and msg.buttons[3]:
            self.vel_d = self.limit_speed(self.vel_d + 0.1)
    
    def timer_cb(self):
        self.vel_d_msg.data = self.vel_d
        self.vel_i_msg.data = self.vel_i
        self.signal_publisher_d.publish(self.vel_d_msg)
        self.signal_publisher_i.publish(self.vel_i_msg)

    def _parameter_callback(self, params):
        for param in params:
            if param.name == "vel_d":
                self.vel_d = self.limit_speed(param.value)
            elif param.name == "vel_i":
                self.vel_i = self.limit_speed(param.value)
            elif param.name == "timer_period":
                self.timer_period = param.value
        return SetParametersResult()
    
def main(args=None):
    rclpy.init(args=args)
    set_point = SetPointPublisher()
    
    try:
        rclpy.spin(set_point)
        
    except KeyboardInterrupt:
        pass
    finally:
        set_point.running = False
        set_point.thread.join()
        set_point.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
