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
        super().__init__("Intel_publisher")  
        self.intel_publisher_rgb = self.create_publisher(Image, "rgb_frame", 10)  # Publisher
        self.intel_publisher_bg_removed = self.create_publisher(Image, "bg_removed_frame", 10)
        self.br_rgb = CvBridge()  # Convertidor de OpenCV a ROS
        
        try:
            # Configuración del pipeline
            self.pipe = rs.pipeline()
            self.cfg = rs.config()
            
            # Enable color and depth
            self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)  # Configuración optimizada
            self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            self.pipe.start(self.cfg)
            
            # Depth Scale
            self.depth_scale = self.pipe.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
            
            # Set clipping distance (1 mts)
            self.clipping_distance = 1.0 / self.depth_scale
            
            # Depth color
            self.align = rs.align(rs.stream.color)
            
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
                
                # Align depth to color frame
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    self.get_logger().warning("No se recibió un frame de color.")
                    continue
                
                # Procesamiento del frame
                color_image = np.asanyarray(color_frame.get_data()) # Esto te lo da en formato BGR
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Convertir a RGB para poder publicar
                color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                self.intel_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(color_image_rgb, encoding='rgb8'))
                
                # Aplicar background removal
                bg_removed = self.background_removal(color_image, depth_image)
                self.intel_publisher_bg_removed.publish(self.br_rgb.cv2_to_imgmsg(bg_removed, encoding='rgb8'))
                
                self.get_logger().info(f"Publishing RGB frame. Procesado en {time.time() - start_time:.3f} segundos")
        except Exception as e:
            self.get_logger().error(f"Error al capturar frames: {e}")

    def background_removal (self, color_image, depth_image) : 
        " Remueve el background con un threshold "
        grey_color = 153
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        
        # Aplicar background removal
        bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        
        # Convertir a de BGR a RGB
        bg_removed_rgb = cv2.cvtColor(bg_removed.astype(np.uint8), cv2.COLOR_BGR2RGB)
        
        return bg_removed_rgb

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