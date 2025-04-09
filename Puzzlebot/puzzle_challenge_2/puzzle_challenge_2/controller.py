#!/usr/bin/env python3

import rclpy, time, math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class ControllerPathNode(Node):
	def __init__(self):
		super().__init__("controller")
		self.get_logger().info("Controller with path node has started")

		self.sub = self.create_subscription(Pose, "/pose", self.callback, 10)
		self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)
		self.create_timer(0.1, self.state_machine)

		self.v = 0.2
		self.w = 0.15

		self.x = 5.5
		self.y = 5.5
		# Pruebas en turtle bot
		#self.x = 5.5
		#self.y = 5.5
		
		self.p_des = Twist()
		self.p_des_ant = Twist()
		self.t_max = 0.0  # tiempo máximo permitido para llegar
		
		self.d_actual = 0.0
		self.d_des = 0.0
		
		self.q_actual = 0.0
		self.q_des = 0.0
		self.q_des_angle = 0.0
		self.q_des_tr = 0.0

		self.state = "stop"  
		self.action_finished = True
	
	def callback(self, msg):
        	self.p_des.linear.x = msg.x 
        	self.p_des.linear.y = msg.y
        	
	def advance(self, desired_distance):
        	msg = Twist()
        	t = time.time() - self.t0
        	distance_traveled = self.v*t
        	if distance_traveled < desired_distance:
        		msg.linear.x = self.v
        		self.pub.publish(msg)
        	else:
        		self.pub.publish(msg)
        		print("distance condition reached")
        		self.action_finished = True
        		
	def turn(self, desired_angle):
        	msg = Twist()
        	t = time.time() - self.t0
        	angle_traveled = self.w*t
        	if abs(angle_traveled) < abs(desired_angle):
        		msg.angular.z = self.w if desired_angle > 0 else -self.w
        		self.pub.publish(msg)
        		
        	else:
        		self.pub.publish(msg)
        		print("angle condition reached")
        		self.action_finished = True
        		
	def state_machine(self):
        	# Transiciones de estado
        	if self.state == "stop" and self.action_finished == True and self.p_des_ant != self.p_des: 
        		self.state = "state1"
        		# Cálculos
        		self.d_des = math.sqrt((self.p_des.linear.y - self.y)**2 + (self.p_des.linear.x - self.x)**2)
        		self.q_des = math.atan2(self.p_des.linear.y - self.y, self.p_des.linear.x - self.x) 
        		print(self.q_des)
        		self.q_des_angle = math.atan2(math.sin(self.q_des), math.cos(self.q_des))
        		# Guardas variables
        		self.p_des_ant.linear.x = self.p_des.linear.x
        		self.p_des_ant.linear.y = self.p_des.linear.y
        		self.action_finished = False
        		self.t0 = time.time()
        		
        	elif self.state == "state1" and self.action_finished == True: 
        		self.q_actual = self.q_des_angle
        		print(self.q_actual)
        		self.state = "state2"
        		self.action_finished = False
        		self.t0 = time.time()
        		
        	elif self.state == "state2" and self.action_finished: 
        		self.d_actual = self.d_des
        		# Guardas variables
        		self.x = self.p_des.linear.x
        		self.y = self.p_des.linear.y
        		self.state = "stop"
        		self.action_finished = True
        		
        	# Ejecutar acciones según estado
        	if self.state == "stop":
        		pass
        	elif self.state == "state1": 
        		self.turn(self.q_des_angle-self.q_actual)
        	elif self.state == "state2":
        		self.advance(self.d_des)

def main():
    rclpy.init()
    nodeh = ControllerPathNode()
    try:
        rclpy.spin(nodeh)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Bye bye")

if __name__ == "__main__":
    main()

