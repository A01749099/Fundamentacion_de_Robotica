#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control
from std_msgs.msg import Float32, String, Bool

class CloseLoopFuzzy(Node):
    def __init__(self):
        super().__init__("close_loop_fuzzy")
        self.get_logger().info("Close loop Fuzzy node with FSM and traffic light started")
		
		# Definición de publicadores y suscriptores
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.sub = self.create_subscription(Float32, "/correccion", self.error_callback, 1)
        self.sub_detection = self.create_subscription(String, "/yolo/detections", self.detection, 5)
        self.sub_light = self.create_subscription(String, "/semaforo/detections", self.light_callback, 5)
        self.sub_zebra = self.create_subscription(Bool, "/zebra_crossing_detected", self.zebra, 5)

        self.create_timer(0.1, self.state_machine)
		
		# Variables para la maquina de estados y guardar valores de suscripciones
        self.state = 'FOLLOW_LINE'
        self.signal = ""
        self.error = 0.0
        self.override_msg = Twist()
        self.t0 = None
        self.wait_duration = 0.0
        self.signal_handled = False
        self.traffic_light = "verde"
		
		# Definición de tiempos para realizar las vueltas
        self.pre_turn_duration = 0.5
        self.turn_duration = 2.0
        self.pre_turn_executed = False

		# Definición del controlador difuso mamdani
        PE = control.Antecedent(np.arange(-1, 1, 0.001), 'PE')
        Vx = control.Consequent(np.arange(0, 0.15, 0.001), 'Vx')
        Wz = control.Consequent(np.arange(-0.65, 0.65, 0.001), 'Wz')

        PE['MuchoNegativo'] = fuzz.zmf(PE.universe, -0.6, -0.3)
        PE['PocoNegativo'] = fuzz.trimf(PE.universe, [-0.3, -0.2, -0.1])
        PE['Ignorable'] = fuzz.trimf(PE.universe, [-0.1, -0.05, 0.1])
        PE['PocoPositivo'] = fuzz.trimf(PE.universe, [0.1, 0.2, 0.3])
        PE['MuchoPositivo'] = fuzz.smf(PE.universe, 0.3, 0.6)

        Vx['Sigue'] = fuzz.smf(Vx.universe, 0.11, 0.13)
        Vx['Vuelta'] = fuzz.trapmf(Vx.universe, [0.07, 0.08, 0.11, 0.13])

        Wz['GiraDerecha'] = fuzz.zmf(Wz.universe, -0.55, -0.5)
        Wz['CorrijeDerecha'] = fuzz.trimf(Wz.universe, [-0.25, -0.15, 0])
        Wz['Nada'] = fuzz.trimf(Wz.universe, [-0.02, 0, 0.02])
        Wz['CorrijeIzquierda'] = fuzz.trimf(Wz.universe, [0, 0.15, 0.25])
        Wz['GiraIzquierda'] = fuzz.smf(Wz.universe, 0.5, 0.55)

        r1 = control.Rule(PE['MuchoNegativo'], (Vx['Vuelta'], Wz['GiraIzquierda']))
        r2 = control.Rule(PE['PocoNegativo'], (Vx['Sigue'], Wz['CorrijeIzquierda']))
        r3 = control.Rule(PE['Ignorable'], (Vx['Sigue'], Wz['Nada']))
        r4 = control.Rule(PE['PocoPositivo'], (Vx['Sigue'], Wz['CorrijeDerecha']))
        r5 = control.Rule(PE['MuchoPositivo'], (Vx['Vuelta'], Wz['GiraDerecha']))

        system = control.ControlSystem([r1, r2, r3, r4, r5])
        self.puzzle = control.ControlSystemSimulation(system)
		
		# Variables para detección de cebras y asignación de señales permitidas
        self.last_signal = ""
        self.line_lost = False
        self.zebra1 = False
        self.executed_signal = False
        self.allowed_loss = {"turn_left", "turn_right", "go_straight"}
        self.prev_zebra = False

	# Guardar variables de los suscriptores
    def error_callback(self, msg):
        self.error = msg.data

    def zebra(self, msg):
        self.zebra1 = msg.data

    def light_callback(self, msg):
        self.traffic_light = msg.data.strip().lower()
        
        if self.traffic_light == "no detections":
        	self.traffic_light= "verde"
	
	# Función para la detección de señales de tránsito
    def detection(self, msg):
        signal = msg.data.strip().lower()
        
        # Solo actualizar si no estamos ejecutando acción (WAIT o PRE_TURN)
        if signal != self.signal and self.can_process_signal():
            if self.state in ['FOLLOW_LINE', 'DETECTION']:
                self.signal = signal
                self.signal_handled = False
                if signal in self.allowed_loss:
                    self.last_signal = signal

                if signal not in self.allowed_loss:
                    self.state = 'DETECTION'
                    self.get_logger().info(f"Señal detectada: {self.signal}")
                else:
                    self.get_logger().info(f"Señal detectada y guardada para futura ejecución: {self.signal}")
            else:
                # para saber que ignoró la señal porque está ejecutando
                self.get_logger().info(f"Ignorando señal '{signal}' porque la acción actual no ha terminado.")

	# Mantener en movimiento al puzzlebot y detectar señales
    def can_process_signal(self):
        return self.traffic_light == "verde"
	
	# Realizar la señal definida
    def execute_detection(self):
        if self.signal_handled:
            return

        msg = Twist()
        if self.signal == "stop":
            self.override_msg = Twist()
            self.wait_duration = 11.0
            self.state = "WAIT"

        elif self.signal == "give_way":
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            self.override_msg = msg
            self.wait_duration = 3.0
            self.state = "WAIT"

        elif self.signal == "go_straight":
            msg.linear.x = 0.2
            msg.angular.z = 0.0
            self.override_msg = msg
            self.wait_duration = 1.5
            self.state = "WAIT"

        elif self.signal == "turn_left" or self.signal == "turn_right":
            self.override_msg = Twist()
            self.override_msg.linear.x = 0.15
            self.override_msg.angular.z = 0.0
            self.wait_duration = self.pre_turn_duration
            self.state = "PRE_TURN"
            self.pre_turn_executed = False
            self.get_logger().info(f"Avanzando antes de {self.signal.upper()}")

        elif self.signal == "roadwork_ahead":
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            self.override_msg = msg
            self.wait_duration = 4.0
            self.state = "WAIT"

        else:
            self.get_logger().info("Señal desconocida, continuar con fuzzy.")
            self.state = "FOLLOW_LINE"
            return
            
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.signal_handled = True
        if self.state != "PRE_TURN":
            self.get_logger().info(f"Ejecutando acción por {self.wait_duration:.1f} segundos: {self.signal.upper()}")
	
	# Definición de la máquina de estados
    def state_machine(self):
        self.get_logger().info(f"Semáforo actual: {self.traffic_light.upper()}, Última señal válida: {self.last_signal}")
        
        # Detener al puzzlebot
        if self.state == "FOLLOW_LINE" and self.traffic_light == "rojo":
            self.pub.publish(Twist())
            self.get_logger().info("Semáforo ROJO: Deteniendo completamente.")
            return
            
        # Detección del cruce peatonal y ejecución de la señal almacenada
        if self.zebra1 and not self.executed_signal:
            if self.last_signal in self.allowed_loss:
                self.signal = self.last_signal
                self.signal_handled = False
                self.get_logger().info(f"Cruce peatonal detectado. Ejecutando señal guardada: {self.signal.upper()}")
                self.execute_detection()
                self.executed_signal = True
            else:
                self.get_logger().info("Cruce peatonal detectado, pero no hay señal válida guardada.")
                
       	# Seguidor de línea
        if self.state == "FOLLOW_LINE":
            msg = Twist()
            self.puzzle.input['PE'] = self.error
            self.puzzle.compute()
            msg.linear.x = self.puzzle.output['Vx']
            msg.angular.z = self.puzzle.output['Wz']

            if not self.zebra1:
                self.executed_signal = False

            if self.traffic_light == "amarillo":
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.get_logger().info("Semáforo AMARILLO: reduciendo velocidad lineal, ignorando señales.")

            self.pub.publish(msg)

        # Ejecutar acciones de las señales excluidas para hacerlas durante el recorrido
        elif self.state == "DETECTION":
            if self.traffic_light == "verde" or self.traffic_light == "No detections":
                if self.signal not in self.allowed_loss:
                    self.execute_detection()
                else:
                    self.get_logger().info(f"Esperando cruce peatonal para ejecutar: {self.signal.upper()}")
            else:
                self.get_logger().info(f"Ignorando señal '{self.signal}' por semáforo: {self.traffic_light}")
                self.state = "FOLLOW_LINE"

        elif self.state == "PRE_TURN":
            self.pub.publish(self.override_msg)
            t = self.get_clock().now().nanoseconds / 1e9 - self.t0
            if t >= self.wait_duration and not self.pre_turn_executed:
                msg = Twist()
                if self.signal == "turn_left":
                    msg.linear.x = 0.15
                    msg.angular.z = 0.7
                elif self.signal == "turn_right":
                    msg.linear.x = 0.15
                    msg.angular.z = -0.65

                self.override_msg = msg
                self.wait_duration = self.turn_duration
                self.t0 = self.get_clock().now().nanoseconds / 1e9
                self.state = "WAIT"
                self.pre_turn_executed = True
                self.get_logger().info(f"Iniciando giro: {self.signal.upper()}")


        # Realizar la reducción de velocidad en movimientos y detener al robot por tiempo establecido
        elif self.state == "WAIT":
            if self.signal == "roadwork_ahead":
                msg = Twist()
                self.puzzle.input['PE'] = self.error
                self.puzzle.compute()
                msg.linear.x = self.puzzle.output['Vx']/2
                msg.angular.z = self.puzzle.output['Wz']/2
                self.pub.publish(msg)
                t = self.get_clock().now().nanoseconds / 1e9 - self.t0
                
            elif self.signal == "stop":
                msg = Twist()
                t = self.get_clock().now().nanoseconds / 1e9 - self.t0
                if t <=5.0:
                	self.puzzle.input['PE'] = self.error
                	self.puzzle.compute()
                	msg.linear.x = self.puzzle.output['Vx']
                	msg.angular.z = self.puzzle.output['Wz']
                	self.pub.publish(msg)
                else:
                	self.pub.publish(self.override_msg)
                	t = self.get_clock().now().nanoseconds / 1e9 - self.t0
                
            elif self.signal == "give_way":
                msg = Twist()
                self.puzzle.input['PE'] = self.error
                self.puzzle.compute()
                msg.linear.x = self.puzzle.output['Vx']/2
                msg.angular.z = self.puzzle.output['Wz']/2
                self.pub.publish(msg)
                t = self.get_clock().now().nanoseconds / 1e9 - self.t0
            else: 
                self.pub.publish(self.override_msg)
                t = self.get_clock().now().nanoseconds / 1e9 - self.t0

            # Reinicio de variables
            if t >= self.wait_duration:
                self.get_logger().info("Fin de espera. Regresando al seguimiento de línea.")
                self.state = "FOLLOW_LINE"

                self.signal = ""
                self.last_signal = ""
                self.executed_signal = False
                self.signal_handled = False
                self.pre_turn_executed = False

                
def main():
    rclpy.init()
    node = CloseLoopFuzzy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

