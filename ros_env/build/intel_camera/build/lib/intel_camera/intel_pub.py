import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud
from sensor_msgs_py.point_cloud2 import PointField
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
import threading
import time


class IntelPublisher(Node):
    def __init__(self):
        super().__init__("Intel_publisher")  
        
        # Publishers
        self.intel_publisher_rgb = self.create_publisher(Image, "rgb_frame", 10)
        self.intel_publisher_bg_removed = self.create_publisher(Image, "bg_removed_frame", 10)
        self.intel_publisher_pcl = self.create_publisher(PointCloud2, "point_cloud", 10)
        
        self.br_rgb = CvBridge()
        
        try:
            # Configuración del pipeline
            self.pipe = rs.pipeline()
            self.cfg = rs.config()
            
            # Habilitar color y profundidad
            self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            self.pipe.start(self.cfg)
            
            # Profundidad y alineación
            self.depth_scale = self.pipe.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
            self.clipping_distance = 1.0 / self.depth_scale
            self.align = rs.align(rs.stream.color)
            
            # Procesamiento de nube de puntos
            self.pc = rs.pointcloud()

            # Hilo para captura de frames
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
                frames = self.pipe.wait_for_frames(5000)  # Timeout de 5 segundos
                
                # Alinear profundidad al color
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    self.get_logger().warning("No se recibió un frame de color.")
                    continue
                
                # Convertir a numpy
                color_image = np.asanyarray(color_frame.get_data())  # Formato BGR
                depth_image = np.asanyarray(depth_frame.get_data())

                # Convertir BGR a RGB para ROS
                color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                self.intel_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(color_image_rgb, encoding='rgb8'))
                
                # Remover background
                bg_removed = self.background_removal(color_image, depth_image)
                self.intel_publisher_bg_removed.publish(self.br_rgb.cv2_to_imgmsg(bg_removed, encoding='rgb8'))
                
                # Generar y publicar PointCloud2
                point_cloud = self.generate_point_cloud(color_frame, depth_frame)
                if point_cloud:
                    self.intel_publisher_pcl.publish(point_cloud)
                
                self.get_logger().info(f"Frames publicados en {time.time() - start_time:.3f} segundos")
        except Exception as e:
            self.get_logger().error(f"Error al capturar frames: {e}")

    def background_removal(self, color_image, depth_image):
        """Remueve el background con un threshold"""
        grey_color = (153, 153, 153)  # Gris en BGR
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        
        # Aplicar filtro de background
        bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        
        # Convertir de BGR a RGB
        return cv2.cvtColor(bg_removed.astype(np.uint8), cv2.COLOR_BGR2RGB)

    def generate_point_cloud(self, color_frame, depth_frame):
        """Convierte los frames en un mensaje PointCloud2"""
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        height, width = depth_frame.get_height(), depth_frame.get_width()

        points = []
        color_image = np.asanyarray(color_frame.get_data())

        for y in range(0, height, 2):  # Procesamos cada 2 píxeles para mejorar rendimiento
            for x in range(0, width, 2):
                depth = depth_frame.get_distance(x, y)
                if depth == 0:  # Ignorar puntos sin datos
                    continue
                
                # Proyectar píxel a espacio 3D
                point = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
                
                # Extraer color en formato RGB
                b, g, r = color_image[y, x]
                points.append([point[0], point[1], point[2], r, g, b])

        if not points:
            self.get_logger().warning("No se generaron puntos en el PointCloud.")
            return None

        # Crear el mensaje PointCloud2
        header = header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera frame"

        # Definir campos para PointCloud2
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="r", offset=12, datatype=PointField.UINT8, count=1),
            PointField(name="g", offset=13, datatype=PointField.UINT8, count=1),
            PointField(name="b", offset=14, datatype=PointField.UINT8, count=1),
        ]

        
        return create_cloud(header, fields, points)

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
