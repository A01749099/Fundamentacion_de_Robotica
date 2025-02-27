import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from my_interfaces.srv import SetProcessBool
from rcl_interfaces.msg import SetParametersResult

# Class Definition
class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # PID Parameters (Adjust these values)
        self.declare_parameter("Kp",0.02255)
        self.declare_parameter("Ki",1.1563)
        self.declare_parameter("Kd",0.00005625)
        
        # Parámetros de la señal globales
        self.Kp = self.get_parameter("Kp").value 
        self.Ki = self.get_parameter("Ki").value 
        self.Kd = self.get_parameter("Kd").value
        
        
        self.Ts = 0.001  # Sample time (same as timer period)

        # Variables for PID
        self.prev_error = 0.0
        self.integral = 0.0

        # Messages
        self.ctrl_output_msg = Float32()

        # Set variables to be used
        self.input_sp = 0.0
        self.output_motor = 0.0
        self.system_running = False

        # Timer period definition
        timer_period = self.Ts  # Tiempo en segundos

        # Creación de suscriptores y publicadores
        self.signal_pub = self.create_publisher(Float32, 'motor_input_u', 10)
        self.motor_speed_sub = self.create_subscription(Float32, 'motor_output_y', self.output_cb_y, 10)
        self.signal_sp_sub = self.create_subscription(Float32, 'set_point', self.input_cb, 10)
        self.timer = self.create_timer(timer_period, self.timer_cb)

        self.add_on_set_parameters_callback(self._parameter_callback)
        

        # Create a service client for /EnableProcess
        self.cli = self.create_client(SetProcessBool, 'EnableProcess')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Enviar solicitud solo después de verificar que el servicio está disponible
        self.send_request(True)

    # Subscriber Callback
    def input_cb(self, input_sgn):
        self.input_sp = input_sgn.data
        
    # Subscriber Callback
    def output_cb_y(self, input_sgn):
        self.output_motor = input_sgn.data
    
    def _parameter_callback(self,params):
        for param in params:
            if param.name == "Kp":
                self.Kp=param.value
            
            elif param.name == "Ki":
                self.Ki=param.value

            elif param.name == "Kd":
                self.Kd=param.value


        return SetParametersResult

    # Timer Callback (Implementing PID)
    def timer_cb(self):

        # Compute error
        error = self.input_sp - self.output_motor

        # Compute PID terms
        P = self.Kp * error
        self.integral += error * self.Ts  # Integral term
        I = self.Ki * self.integral
        D = self.Kd * (error - self.prev_error) / self.Ts  # Derivative term

        # Compute control output
        u = P + I + D

   
        # Store previous error
        self.prev_error = error

        # Publish control signal
        self.ctrl_output_msg.data = u
        self.signal_pub.publish(self.ctrl_output_msg)

    def send_request(self, enable: bool):
        """Send a request to start or stop the simulation."""
        request = SetProcessBool.Request()
        request.enable = enable

        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """Process the service response."""
        try:
            response = future.result()
            if response.success:
                self.system_running = True
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.system_running = False
                self.get_logger().warn(f'Failure: {response.message}')
        except Exception as e:
            self.system_running = False
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

# Execute Node
if __name__ == '__main__':
    main()
