import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
import threading
import time


class IntelPublisher(Node):
    def __init__(self):
        super().__init__("Intel_publisher")  # Nombre corregido
        self.intel_publisher_rgb = self.create_publisher(Image, "rgb_frame", 10)  # Publicador
        self.br_rgb = CvBridge()  # Convertidor de OpenCV a ROS
        
        try:
            # Configuración del pipeline
            self.pipe = rs.pipeline()
            self.cfg = rs.config()
            self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)  # Configuración optimizada
            self.pipe.start(self.cfg)
            
            # Hilo separado para el procesamiento de frames
            self.camera_thread = threading.Thread(target=self.start_camera, daemon=True)
            self.camera_thread.start()
            
            self.get_logger().info("Intel RealSense inicializada correctamente.")
        except Exception as e:
            self.get_logger().error(f"Error inicializando la cámara Intel RealSense: {e}")

    def start_camera(self):
        """Hilo separado para procesar frames."""
        try:
            while rclpy.ok():
                start_time = time.time()
                frames = self.pipe.wait_for_frames(5000)  # Tiempo de espera de 5 segundos
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    self.get_logger().warning("No se recibió un frame de color.")
                    continue
                
                # Procesamiento del frame
                color_image = np.asanyarray(color_frame.get_data())
                self.intel_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(color_image))
                self.get_logger().info(f"Publishing RGB frame. Procesado en {time.time() - start_time:.3f} segundos")
        except Exception as e:
            self.get_logger().error(f"Error al capturar frames: {e}")

    def destroy_node(self):
        """Detiene el pipeline al cerrar el nodo."""
        self.pipe.stop()
        self.get_logger().info("Pipeline de Intel RealSense detenido.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    intel_publisher = IntelPublisher()
    try:
        rclpy.spin(intel_publisher)
    except KeyboardInterrupt:
        intel_publisher.get_logger().info("Nodo interrumpido por el usuario.")
    finally:
        intel_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
