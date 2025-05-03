#!/usr/bin/env python3

import cv2, rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

class OpenCVBridge(Node):
    def __init__(self):
        super().__init__('procesing_node')
        self.get_logger().info('Node has started')
        self.img=None
        self.bridge=CvBridge()
        self.sub=self.create_subscription(Image,"/video_source/raw", self.camera_callback,10)
        self.pub = self.create_publisher(Int32, "/light", 10)
        self.timer=self.create_timer(0.1,self.timer_callback)

        # Define HSV ranges for traffic light colors
        # Red has two ranges in HSV (wraps around 180)
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])
        
        # Yellow
        self.lower_yellow = np.array([10, 100, 122])
        self.upper_yellow = np.array([30, 255, 150])
        self.lower_yellow2 = np.array([23, 0, 193])
        self.upper_yellow2 = np.array([40, 255, 245])
        
        # Green
        self.lower_green = np.array([40, 50, 50])
        self.upper_green = np.array([80, 255, 255])
        
        self.mask_red = 0
        self.mask_green = 0
        self.mask_yellow = 0
        self.num = 3

    def camera_callback(self,msg):
        #self.img= self.bridge.imgmsg_to_cv2(msg,"mono8")   #Opcion 1 bgr8
        self.img= self.bridge.imgmsg_to_cv2(msg,"bgr8")

    

    def sliders(self):
        def empty(a): pass
        window_name= 'HSV Ranges'
        cv2.imshow(window_name,np.zeros((10,650,3),np.uint8))
        cv2.createTrackbar('Hue Min',window_name,0,180,empty)
        cv2.createTrackbar('Hue Max',window_name,180,180,empty)
        cv2.createTrackbar('Sat Min',window_name,0,255,empty)
        cv2.createTrackbar('Sat Max',window_name,255,255,empty)
        cv2.createTrackbar('Val Min',window_name,0,255,empty)
        cv2.createTrackbar('Val Max',window_name,255,255,empty)

        hsv=cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)
        while(True):
            #get the values from the trackbars
            hmin=cv2.getTrackbarPos('Hue Min',window_name)
            hmax=cv2.getTrackbarPos('Hue Max',window_name)
            smin=cv2.getTrackbarPos('Sat Min',window_name)
            smax=cv2.getTrackbarPos('Sat Max',window_name)
            vmin=cv2.getTrackbarPos('Val Min',window_name)
            vmax=cv2.getTrackbarPos('Val Max',window_name)

            #Get image in the range
            lower_range=np.array([hmin,smin,vmin])
            upper_range=np.array([hmax,smax,vmax])
            treshold=cv2.inRange(hsv,lower_range,upper_range)
            bitwise=cv2.bitwise_and(self.img,self.img,mask=treshold)
            cv2.imshow("Result",bitwise)
            cv2.imshow("Treshold",treshold)
            cv2.imshow(window_name,hsv)
            cv2.waitKey(1) & 0xFF
            
        cv2.destroyAllWindows()




    def detect_traffic_light_color(self, hsv_frame, frame):
        height, width, _ = frame.shape

        # Define ROI en la parte superior central (puedes ajustar estos valores)
        self.roi_y_start = int(height * 0.1)
        self.roi_y_end = int(height * 0.4)
        self.roi_x_start = int(width * 0.4)
        self.roi_x_end = int(width * 0.6)
        
        self.roi = hsv_frame[self.roi_y_start:self.roi_y_end, self.roi_x_start:self.roi_x_end]

        # Máscaras de color en la ROI
        mask_red1 = cv2.inRange(self.roi, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(self.roi, self.lower_red2, self.upper_red2)
        self.mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        mask_yellow1 = cv2.inRange(self.roi, self.lower_yellow, self.upper_yellow)
        mask_yellow2 = cv2.inRange(self.roi, self.lower_yellow2, self.upper_yellow2)

        self.mask_yellow=cv2.inRange(self.roi, self.lower_yellow, self.upper_yellow)

        #self.mask_yellow = cv2.bitwise_or(mask_yellow1, mask_yellow2)

        self.mask_green = cv2.inRange(self.roi, self.lower_green, self.upper_green)

        # Conteo de píxeles
        red_count = cv2.countNonZero(self.mask_red)
        yellow_count = cv2.countNonZero(self.mask_yellow)
        green_count = cv2.countNonZero(self.mask_green)

        # Determinar color dominante
        max_color = max(red_count, yellow_count, green_count)
        self.color = "NONE"
        if max_color > 300:  # Umbral de detección
            if max_color == red_count:
                self.color = "RED"
                self.num = 2
            elif max_color == yellow_count:
                self.color = "YELLOW"
                self.num = 1
            elif max_color == green_count:
                self.color = "GREEN"
                self.num = 0

        # Mostrar máscaras
        #cv2.imshow("Red Mask", cv2.resize(self.mask_red, (200, 150)))
        #cv2.imshow("Yellow Mask", cv2.resize(self.mask_yellow, (200, 150)))
        #cv2.imshow("Green Mask", cv2.resize(self.mask_green, (200, 150)))

        return self.color, self.roi

    def timer_callback(self):
        
        if self.img is not None:
            frame=self.img.copy()
            hsv_frame= cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            height,width,_=frame.shape

            cx=int(width/2)
            cy=int(height/2)

           # Detect traffic light color
            color, self.roi = self.detect_traffic_light_color(hsv_frame, frame)

            # Display info
            cv2.rectangle(frame, (self.roi_x_start, self.roi_y_start), (self.roi_x_end, self.roi_y_end), (255, 255, 255), 2)

            # Set text color based on detected color
            text_color = (0, 0, 0)  # Default black
            if color == "RED":
                text_color = (0, 0, 255)  # BGR: Bright red
            elif color == "YELLOW":
                text_color = (0, 255, 255)  # BGR: Yellow
            elif color == "GREEN":
                text_color = (0, 255, 0)  # BGR: Green
                
            self.last_color = self.num
            msg = Int32()
            msg.data = self.last_color
            self.pub.publish(msg)
            
            cv2.putText(frame, color, (cx-30, 20), 0, 0.8, text_color, 2)
            
            # Redimensionamos todas las imágenes
            frame_resized = cv2.resize(frame, (320, 240))
            mask_red_resized = cv2.resize(self.mask_red, (320, 240))
            mask_yellow_resized = cv2.resize(self.mask_yellow, (320, 240))
            mask_green_resized = cv2.resize(self.mask_green, (320, 240))
            
            # Convertimos las máscaras a BGR para poder concatenarlas
            mask_red_color = cv2.cvtColor(mask_red_resized, cv2.COLOR_GRAY2BGR)
            mask_yellow_color = cv2.cvtColor(mask_yellow_resized, cv2.COLOR_GRAY2BGR)
            mask_green_color = cv2.cvtColor(mask_green_resized, cv2.COLOR_GRAY2BGR)
            
            # Poner textos sobre las imágenes
            cv2.putText(frame_resized, "FRAME", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
            cv2.putText(mask_red_color, "RED MASK", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(mask_yellow_color, "YELLOW MASK", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(mask_green_color, "GREEN MASK", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Juntamos imágenes
            top_row = cv2.hconcat([frame_resized, mask_red_color])
            bottom_row = cv2.hconcat([mask_yellow_color, mask_green_color])
            final_image = cv2.vconcat([top_row, bottom_row])
            
            # Mostrar una sola ventana
            cv2.imshow("Traffic Light Detection", final_image)
            cv2.waitKey(1)

           

def main(args=None):
    rclpy.init(args=args)
    nodeh=OpenCVBridge()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Adios")

if __name__=='__main__':
    main()