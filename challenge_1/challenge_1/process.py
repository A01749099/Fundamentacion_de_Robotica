#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import numpy as np

class MyClassNodes(Node):
    def __init__(self):
        super().__init__("process") #Nombre de nodo

        #Escucha de los topicos signal y time
        self.get_logger().info("Listener node has started")
        self.listener_signal = self.create_subscription(Float32,"signal",self.callback_signal,10)
        self.listener_time = self.create_subscription(Float32,"time",self.callback_time,10)
        
        #Publica en el topico proc__signal
        self.signal_proc = self.create_publisher(Float32, "proc_signal", 10)

        # Parámetros de la señal globales
        self.o = 0.0       # Fase
        self.K = 1.0       # Offset
        
        # Variables para guardar señal y tiempo
        self.received_signal = None
        self.received_time = None
    
    #Imprime valores de los topicos escuchados anteriormente
    def callback_signal(self,msg):
        self.get_logger().info(f"Signal value: {msg.data}")
        self.received_signal=msg.data
        self.processed_signal()
    
    def callback_time(self,msg):
        self.get_logger().info(f"Time value: {msg.data}")
        self.received_time=msg.data

    #Genera la nueva señal procesada con nuevos parametros
    def processed_signal(self):
        if self.received_signal is None or self.received_time is None:
            return  # No seguir si aún no obtiene la señal o el tiempo


        offset = self.received_signal + self.K  #Aplicar Offset
        t=self.received_time    #Tiempo escuchado
        process_signal=offset/2   #Reducir amplitud a la mitad

        msg=Float32()
        msg.data = process_signal
        
        self.get_logger().info(f"New Signal : {msg.data}")
        self.signal_proc.publish(msg)


def main():
    rclpy.init()
    nodeh=MyClassNodes()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Adios")

if __name__== "__main__":
    main()
