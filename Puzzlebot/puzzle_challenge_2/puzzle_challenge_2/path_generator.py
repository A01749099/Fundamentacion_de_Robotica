#!/usr/bin/env python3

#importaciones
import rclpy, time
from rclpy.node import Node
from turtlesim.msg import Pose

#Clase
class PathGeneratorClass(Node):
	def __init__(self):
		super().__init__("path_generator")
		self.get_logger().info("Path generator node has been started ...")
		self.create_timer(0.1, self.generator_callback)
		self.pub = self.create_publisher(Pose, "/pose", 1)
		self.t0 = time.time()
		self.msg = Pose()
		#self.point_list = [[2.0,0.0,11.0], [2.0,2.0,33.0], [0.0,2.0,56.0], [0.0,0.0,98.0], [2.0,2.0,130.0], [1.0,3.0,140.0], [0.0,2.0,176.0], [0.0,3.0,230.0]] # Point to visit [x,y,t]
		self.point_list = [[7.5,5.5,11.0], [7.5,7.5,33.0], [5.5,7.5,56.0], [5.5,5.5,98.0],  [7.5,7.5,130.0], [6.5,8.5,150.0], [5.5,7.5,196.0], [5.5, 8.5, 230.0]] # Point to visit [x,y,t]
		
	def generator_callback(self):
		if len(self.point_list) > 0:
			elapsed_time = time.time()-self.t0
			print("Elapsed time = %.3f"%elapsed_time)
			[x,y,t] = self.point_list[0] # Extract the first point
			if elapsed_time < t:
				self.msg.x = x
				self.msg.y = y
				self.pub.publish(self.msg)
			else:
				self.point_list.pop(0)
		else:
			print("End of list")
			rclpy.shutdown()
#Funcion main
def main(args=None):

	#Conexion a ros
	rclpy.init(args=args)
	nodeh = PathGeneratorClass() #Instancia
	
	try: rclpy.spin(nodeh)
	
	#Excepciones
	except Exception as error: print(error)
	except KeyboardInterrupt: print("Node terminaterd by user")
	
if __name__=="__main__":
	main()
