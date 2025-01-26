import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud
from sensor_msgs_py.point_cloud2 import PointField
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2
import pyrealsense2 as rs
import numpy as np
import threading
import time


class IntelPublisher(Node):
    def __init__(self):
        super().__init__("Intel_publisher")  # Nombre corregido
        self.intel_publisher_rgb = self.create_publisher(Image, "rgb_frame", 10)  # Publicador
        self.intel_publisher_pcl = self.create_publisher(PointCloud2, "Point_cloud", 10)
        self.br_rgb = CvBridge()  # Convertidor de OpenCV a ROS
        
        try:
            # Configuración del pipeline
            self.pipe = rs.pipeline()
            self.cfg = rs.config()
            self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)  # Configuración optimizada
            self.cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16, 15) # Depth Stream
            self.pipe.start(self.cfg)
            
            # Get depth
            self.depth_scale = self.pipe.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
                        
            # Hilo separado para el procesamiento de frames
            self.camera_thread = threading.Thread(target=self.start_camera, daemon=True)
            self.camera_thread.start()
            
            self.get_logger().info("Intel RealSense inicializada correctamente.")
        except Exception as e:
            self.get_logger().error(f"Error inicializando la cámara Intel RealSense: {e}")


    def start_camera(self):
        """Thread to process frames."""
        try:
            while rclpy.ok():
                start_time = time.time()
                try:
                    frames = self.pipe.wait_for_frames(5000)  # 5-second timeout
                except Exception as e:
                    self.get_logger().warning(f"Frame capture timeout: {e}")
                    continue

                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    self.get_logger().warning("No color or depth frame received.")
                    continue

                # Convert frames to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())

                # Publish RGB frame
                self.intel_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(color_image, encoding="bgr8"))

                # Publish point cloud
                point_cloud = self.create_point_cloud(color_frame, depth_frame)
                self.intel_publisher_pcl.publish(point_cloud)

                self.get_logger().info(f"Frames processed in {time.time() - start_time:.3f} seconds")
        except Exception as e:
            self.get_logger().error(f"Error capturing frames: {e}")


    def create_point_cloud(self, color_frame, depth_frame):
        """Generate PointCloud2 message from depth and color frames."""
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        height, width = depth_frame.get_height(), depth_frame.get_width()

        points = []
        color_image = np.asanyarray(color_frame.get_data())

        for y in range(height):
            for x in range(width):
                depth = depth_frame.get_distance(x, y)
                if depth == 0:  # Skip invalid points
                    continue

            # Project pixel to 3D space
                point = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)

            # Add color to the point
                b, g, r = color_image[y, x]  # OpenCV uses BGR format
                points.append([point[0], point[1], point[2], r, g, b])

        if not points:
            self.get_logger().warning("No points generated for point cloud.")
            return None

        self.get_logger().info(f"Number of points in point cloud: {len(points)}")

    # Define PointField for PointCloud2
        fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="r", offset=12, datatype=PointField.UINT8, count=1),
        PointField(name="g", offset=13, datatype=PointField.UINT8, count=1),
        PointField(name="b", offset=14, datatype=PointField.UINT8, count=1),
    ]

    # Create the header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_frame"


    # Generate PointCloud2
        return create_cloud(header, fields, points) # Needs a header 
        #return create_cloud(fields, points)




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
        intel_publisher.get_logger().info("Node interrupted by user.")
    finally:
        intel_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
