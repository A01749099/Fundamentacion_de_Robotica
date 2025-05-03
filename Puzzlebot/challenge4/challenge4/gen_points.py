#!/usr/bin/env python3
import rclpy,time
from rclpy.node import Node
from turtlesim.msg import Pose

class PathGeneratorClass(Node):
    def __init__(self):
        super().__init__("path_gener") #Conectar a ros con nodo
        self.create_timer(0.1,self.generator_callback)
        self.get_logger().info("Path generator node has been started")
        self.pub = self.create_publisher(Pose, "pose", 1)
        self.point_list = [[1.5,0.0,10.0], [1.5,1.5,20.0], [0.0,1.5,30.0], [0.0,0.0,40.0]] # Point to visit [x,y,t]
        self.t0=time.time()
        self.msg=Pose()

    def generator_callback(self):
        if len(self.point_list)>0:
            elapsed_time=time.time()-self.t0
            print("Elapsed time= %.3f"%elapsed_time)
            [x,y,t]=self.point_list[0] #Extract first point
            if elapsed_time<t:
                self.msg.x=x
                self.msg.y=y
                self.pub.publish(self.msg)
            else:
                self.point_list.pop(0)
        else:
            print("End of list")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args) #Conectar a ros
    nodeh = PathGeneratorClass()
    try : rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user")

    #rclpy.shutdown()

if __name__== "__main__":
    main()
