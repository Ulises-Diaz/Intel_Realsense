import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from cv_bridge import CvBridge
import cv2
import numpy as np

class IntelSubscriber(Node) : 
    def __init__ (self) : 
        super().__init__("Intel_subscriber")
        self.subscription_rgb = self.create_subscription(Image, "rgb_frame", self.rgb_frame_callback, 10)
        self.subscription_bg_removed = self.create_subscription(Image, "bg_removed_frame", self.bg_removed_callback, 10)
        self.subscription_point_cloud = self.create_subscription(PointCloud2, "point_cloud", self.point_cloud_callback, 10)
        
        self.br_rgb = CvBridge()

    def rgb_frame_callback(self, data): 
        self.get_logger().warning("Receiving RGB frame")
        current_frame = self.br_rgb.imgmsg_to_cv2(data) 
        cv2.imshow("RGB", current_frame)
        cv2.waitKey(1)

    def bg_removed_callback(self, data): 
        self.get_logger().info("Recieving Background-Removed Frame")
        bg_removed_frame = self.br_rgb.imgmsg_to_cv2(data, desired_encoding= "rgb8")
        cv2.imshow("Background removed Frame" , bg_removed_frame)
        cv2.waitKey(1)

    def point_cloud_callback (self, data) :
        self.get_logger().info("Recieving Point Cloud topic")
        
        #Convertir PC a lista de puntos
        points = np.array([
            [point[0], point[1], point[2], point[3], point[4], point[5]]  # x, y, z, r, g, b        ])
            for point in read_points(data, field_names = ("x", "y", "z", "r", "g", "b"), skip_nans = True)
        ], dtype = np.float32)
            
        if points.size == 0:
            self.get_logger().warning("PC2 esta vacio ")
            return 
        
        self.get_logger().info(f"PointCloud2 received with {points.shape[0]} points")
        
        # Convertir Pc a imagen 2D 
        image = self.visualize_point_cloud(points)
        cv2.imshow("PC proyeccion", image)
        cv2.waitKey(1)
        
    def visualize_point_cloud (self, points): 
        x = points [:, 0]
        y = points [:, 1]
        
        # Crea una imagen en negro
        img = np.zeros((480, 640), dtype= np.uint8)
        
        # Normalizar y escalar puntos
        x_norm = ((x - np.min(x)) / (np.max(x) - np.min(x)) * (img.shape[1] - 1)).astype(int)
        y_norm = ((y - np.min(y)) / (np.max(y) - np.min(y)) * (img.shape[0] - 1)).astype(int)
    
        img[y_norm, x_norm] = 255  
        
        return img        
        
            
def main (args = None) : 
    rclpy.init(args = args)
    intel_subscriber = IntelSubscriber()
    rclpy.spin(intel_subscriber)
    intel_subscriber.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__":
    main()