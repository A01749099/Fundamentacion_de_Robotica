#!/usr/bin/env python3

import rclpy, math, tf_transformations
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy import qos
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

def euler_from_quaternion(quaternion):
    return tf_transformations.euler_from_quaternion(quaternion)

class controller(Node):
    def __init__(self):
        super().__init__("Controller")
        self.get_logger().info("Close loop node has started")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.sub3 = self.create_subscription(Pose, "/pose", self.path_callback, 1)
        self.sub2 = self.create_subscription(Int32, "/light", self.semaforo_callback, 1)
        self.sub = self.create_subscription(Odometry, "/odom", self.pose_callback, qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.go_to_point)
        self.create_timer(0.1, self.state_machine)
        self.x = None
        self.y = None
        self.q = None 
        self.xd = None
        self.yd = None
        self.state = "stop" 
        self.action_finished = True
        self.n=0
        self.color=0
        self.point_list = [[1.0,0.0], [1.0,1.0], [0.0,1.0], [0.0,0.0]] # Point to visit [x,y,t]

        

    def path_callback(self, msg):
        self.xd = msg.x 
        self.yd = msg.y

    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.q = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def semaforo_callback(self,msg):
        self.color=msg.data

    def go_to_point(self):
        Kv, Kw = 0.2, 0.5
        if self.x is not None and self.xd is not None :
            msg = Twist()
            diff_x = self.xd - self.x
            diff_y = self.yd - self.y
            distance = math.sqrt(diff_x**2 + diff_y**2)
            if distance > 0.1:
                angle = math.atan2(diff_y, diff_x) - self.q
                angle = math.atan2(math.sin(angle), math.cos(angle))
                msg.linear.x = 0.0 if abs(angle) > 0.2 else Kv
                if (self.color==0 or self.color==3):    #Semaforo Verde
                    msg.angular.z = Kw * angle
                    self.action_finished = False
                    print("Estoy verde")
                elif(self.color==1):                    #Semaforo Amarrillo
                    msg.linear.x = 0.0 if abs(angle) > 0.2 else (Kv/2)
                    msg.angular.z = (Kw/2) * angle
                    self.action_finished = False
                    print("Estoy amarrillo")
                elif self.color==2:                     #Semaforo Rojo
                    msg.linear.x = 0.0 if abs(angle) > 0.2 else 0.0
                    msg.angular.z = 0.0 * angle
                    self.action_finished = False
                    print("Estoy rojo")
            else:
                self.action_finished = True
            self.pub.publish(msg)
            #self.action_finished = True

        

    def state_machine(self):
        
        if self.state == "stop" and self.action_finished:
            if len(self.point_list)>0:
                [x,y]=self.point_list.pop(0) #Extract first point
                self.xd=x
                self.yd=y
                print(f"Target: ({self.xd}, {self.yd})")
               # self.point_list.pop(0)
                self.state = "state1"
                self.action_finished = False
                
            
        elif self.state == "state1" and not self.action_finished:
            self.state = "stop"
            self.action_finished = True
            

        if self.state == "stop":
            pass
        elif self.state == "state1":
            self.go_to_point()

def main():
    rclpy.init()
    nodeh = controller()
    try:
        rclpy.spin(nodeh)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Bye bye")

if __name__ == "__main__":
    main()
