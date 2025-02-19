#!/usr/bin/env python3
import rclpy, sys
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import numpy as np
import math

class MyClassNodes(Node):
    def __init__(self,var):
        print(var)
        super().__init__("signal_generator")
        self.get_logger().info("Signal generator has started.")

        self.create_timer(0.1, self.timer_callback)
        self.signal_pub = self.create_publisher(Float32, "signal", 10)
        self.tm_pub = self.create_publisher(Float32, "time", 10)

        # Guardar el tiempo de inicio
        self.t0 = time.time()

        # Parámetros de la señal globales
        self.A = float(var[1])       # Amplitud
        self.f = float(var[2])       # Fase
        self.P = float(var[3])      # Periodo
        self.K = float(var[4])       # Offset
        self.W = 2*math.pi/self.P  # Frecuencia angular
        
    def timer_callback(self):
        msg = Float32()
        tm = Float32()

        t = time.time() - self.t0  # Tiempo relativo desde el inicio
        tm.data = t
        msg.data = self.A*math.sin(self.W*t + self.f) + self.K

        self.signal_pub.publish(msg)
        self.tm_pub.publish(tm)

def main(args=None):
    if len(sys.argv)<5:
        print("Error. Falta argumentos")
        print()
    rclpy.init(args=args)
    nodeh = MyClassNodes(sys.argv)

    try : rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user")

if __name__ == "__main__":
    main()
