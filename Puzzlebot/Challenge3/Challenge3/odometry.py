#!/usr/bin/env python3
import rclpy,time,math,tf_transformations
from rclpy.node import Node
from rclpy import qos
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class Class_Odometry(Node):
    def __init__(self):
        super().__init__("odometry_node") #Conectar a ros con nodo
        
        self.get_logger().info("Odometry node has been started")
        self.create_timer(0.1,self.odometry_callback)
        self.pub = self.create_publisher(Odometry, "/odom", 1)
        self.subD = self.create_subscription(Float32, "/VelocityEncR", self.wR_cb,qos.qos_profile_sensor_data)
        self.subL = self.create_subscription(Float32, "/VelocityEncL", self.wL_cb,qos.qos_profile_sensor_data)
        
        self.wR=0.0
        self.wL=0.0
        self.x=0.0
        self.y=0.0
        self.q=0.0
        self.r=0.05
        self.L=0.19

        self.t0=time.time()


    def odometry_callback(self):
        elapsed_time=time.time()-self.t0
        self.t0=time.time()
        v=(self.wR+self.wL)*self.r/2
        w=(self.wR-self.wL)*self.r/self.L
        self.x += v*math.cos(self.q)*elapsed_time
        self.y += v*math.sin(self.q)*elapsed_time
        self.q += w*elapsed_time

        msg=Odometry()
        msg.header.stamp= self.get_clock().now().to_msg()
        msg.header.frame_id= "odom"
        msg.child_frame_id = 'Base_footprint'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        quaternion= tf_transformations.quaternion_from_euler(0,0,self.q)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        self.pub.publish(msg)


    def wR_cb(self,msg):
        self.wR=msg.data

    def wL_cb(self,msg):
        self.wL=msg.data


def main(args=None):
    rclpy.init(args=args) #Conectar a ros
    nodeh = Class_Odometry()
    try : rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user")

    #rclpy.shutdown()

if __name__== "__main__":
    main()