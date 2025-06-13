import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.model = YOLO('/home/daniel/Downloads/bueno.pt')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage,
            '/yolo/image_raw/compressed',  # Tópico de entrada
            self.image_callback,
            10)
        
        self.image_publisher = self.create_publisher(
            Image,
            '/yolo/image_detected',  # Tópico de salida de imagen
            10)

        self.detection_publisher = self.create_publisher(
            String,
            '/yolo/detections',  # Tópico de salida de detecciones en texto
            10)

        self.get_logger().info("YOLO Detector Node has started.")

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Detectar objetos con YOLO
            results = self.model.predict(frame, conf=0.90, imgsz=640)

            # Dibujar los resultados
            annotated_frame = results[0].plot()

            # Convertir OpenCV a ROS Image
            output_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            output_msg.header = msg.header  # Copia el encabezado original
            self.image_publisher.publish(output_msg)

            # Obtener etiquetas detectadas
            names = self.model.names
            detections = results[0].boxes.cls.tolist()
            labels = [names[int(cls_id)] for cls_id in detections]

            # Crear string de salida
            detection_str = ', '.join(labels) if labels else "No detections"
            detection_msg = String()
            detection_msg.data = detection_str
            self.detection_publisher.publish(detection_msg)

            self.get_logger().info(f"Detections: {detection_str}")

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


