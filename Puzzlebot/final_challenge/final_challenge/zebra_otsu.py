#!/usr/bin/env python3

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower_node")
        self.get_logger().info("Nodo de seguimiento de línea iniciado.")
        self.bridge = CvBridge()
        self.image = None

        # Suscripción a la cámara
        self.sub = self.create_subscription(Image, "/line_follower/image_raw", self.image_callback, 10)

        # Publicación del error
        self.pub_error = self.create_publisher(Float32, "/correccion", 10)

        # Publicación del estado del cruce peatonal
        self.pub_zebra = self.create_publisher(Bool, "/zebra_crossing_detected", 10)

        # Temporizador
        self.timer = self.create_timer(0.1, self.timer_callback)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def timer_callback(self):
        if self.image is None:
            return

        direccion, frame_procesado, binaria, error = self.detectar_direccion(self.image)

        # Detectar paso de cebra
        cruce_detectado = self.detectar_cruce_peatonal(binaria)
        msg_zebra = Bool()
        msg_zebra.data = cruce_detectado
        self.pub_zebra.publish(msg_zebra)

        if cruce_detectado:
            self.get_logger().info("Cruce peatonal detectado")
            print("Cruce peatonal detectado")
        else:
            print("Sin cruce peatonal")

        if direccion == "Línea no detectada":
            self.get_logger().info("Línea no detectada")
        else:
            self.get_logger().info(f"Dirección: {direccion} | Error: {error}")
            if error != 0:
                msg = Float32()
                msg.data = float(error)
                self.pub_error.publish(msg)

    def detectar_direccion(self, frame):
        frame = cv2.resize(frame, (320, 180))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        print("aaaaddd")
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        # Opening para limpiar ruido
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        altura = thresh.shape[0]
        ancho = thresh.shape[1]

        # Zona inferior
        zona_alta = altura - 20
        zona = thresh[zona_alta:altura, :]

        # Zona central recortada
        ancho_cuadro = 120
        inicio_x = (ancho - ancho_cuadro) // 2
        fin_x = inicio_x + ancho_cuadro
        zona_central = zona[:, inicio_x:fin_x]

        M = cv2.moments(zona_central)
        if M["m00"] == 0:
            return "Línea no detectada", frame, thresh, 0

        cx_local = int(M["m10"] / M["m00"])
        cx = cx_local + inicio_x
        centro = ancho // 2

        if cx < ancho / 1.5:
            direccion = "Izquierda"
            error = (cx - centro) / 100
        elif cx > 2 * ancho / 3.7:
            direccion = "Derecha"
            error = (cx - centro) / 100
        else:
            direccion = "Recto"
            error = 0

        return direccion, frame, thresh, error

    def detectar_cruce_peatonal(self, binary_image):
        altura, ancho = binary_image.shape
        zona = binary_image[int(altura * 0.5):altura, :]

        contornos, _ = cv2.findContours(zona, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lineas_horizontales = []

        # Detecciones de lineas horizontales para la detección
        for cnt in contornos:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = w / float(h)
            if 2.0 < aspect_ratio < 15.0 and 5 < h < 25 and w > 30:
                lineas_horizontales.append((x, y, w, h))

        if len(lineas_horizontales) >= 3:
            lineas_horizontales.sort(key=lambda item: item[1])
            espaciados = [
                lineas_horizontales[i + 1][1] - lineas_horizontales[i][1]
                for i in range(len(lineas_horizontales) - 1)
            ]
            if max(espaciados) - min(espaciados) < 15:
                return True

        return False

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Nodo detenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

