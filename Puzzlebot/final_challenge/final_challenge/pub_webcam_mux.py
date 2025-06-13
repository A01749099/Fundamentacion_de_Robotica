#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
from cv_bridge import CvBridge

class VideoMultiplexer(Node):
    def __init__(self):
        super().__init__('video_multiplexer')

        # Imagen recibida del nodo de cámara
        self.ima = None
        self.bridge = CvBridge()

        # Suscripción a la fuente principal de video
        self.sub = self.create_subscription(
            Image,
            '/video_source/raw',
            self.callback,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        )

        # Publicadores para cada video
        self.pub_line = self.create_publisher(Image, '/line_follower/image_raw', 10)

        # QoS para nodos de menor prioridad
        yolo_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        

        # Publicador de imagen comprimida para YOLO
        self.pub_yolo_compressed = self.create_publisher(CompressedImage, '/yolo/image_raw/compressed', 10)

        self.pub_semaforo_compressed = self.create_publisher(CompressedImage, '/semaforo/image_raw/compressed', 10)

        # Timers para publicar a cada canal
        self.create_timer(0.05, self.publi_line)      # 20 Hz
        self.create_timer(0.3, self.publi_signal)     # 3 Hz
        self.create_timer(0.3, self.publi_semaforo)
        self.get_logger().info("VideoMultiplexer iniciado correctamente")

    def callback(self, msg):
        self.ima = msg

    def publi_line(self):
        if self.ima is not None:
            self.pub_line.publish(self.ima)

    def publi_signal(self):
        if self.ima is not None and self.pub_yolo_compressed.get_subscription_count() > 0:
            try:
                # Convertir imagen ROS a OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(self.ima, desired_encoding='bgr8')

                # Comprimir a JPEG
                success, encoded_image = cv2.imencode('.jpg', cv_image)
                if not success:
                    self.get_logger().error("No se pudo comprimir la imagen.")
                    return

                # Crear mensaje CompressedImage
                compressed_msg = CompressedImage()
                compressed_msg.header = self.ima.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = encoded_image.tobytes()

                # Publicar imagen comprimida
                self.pub_yolo_compressed.publish(compressed_msg)
                #self.pub_semaforo_compressed.publish(compressed_msg)

            except Exception as e:
                self.get_logger().error(f"Error al comprimir imagen: {str(e)}")
    def publi_semaforo(self):
        if self.ima is not None and self.pub_semaforo_compressed.get_subscription_count() > 0:
            try:
                # Convertir imagen ROS a OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(self.ima, desired_encoding='bgr8')

                # Comprimir a JPEG
                success, encoded_image = cv2.imencode('.jpg', cv_image)
                if not success:
                    self.get_logger().error("No se pudo comprimir la imagen.")
                    return

                # Crear mensaje CompressedImage
                compressed_msg = CompressedImage()
                compressed_msg.header = self.ima.header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = encoded_image.tobytes()

                # Publicar imagen comprimida
                self.pub_semaforo_compressed.publish(compressed_msg)

            except Exception as e:
                self.get_logger().error(f"Error al comprimir imagen: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = VideoMultiplexer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

