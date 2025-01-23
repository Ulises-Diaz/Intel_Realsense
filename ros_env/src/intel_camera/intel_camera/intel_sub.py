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
        self.subscription_pcl = self.create_subscription(PointCloud2, "point_cloud", self.point_cloud_callback, 10)
        self.br_rgb = CvBridge()

    def rgb_frame_callback(self, data): 
        self.get_logger().warning("Receiving RGB frame")
        current_frame = self.br_rgb.imgmsg_to_cv2(data) 
        cv2.imshow("RGB", current_frame)
        cv2.waitKey(1)
        
    def point_cloud_callback (self, data): 
        self.get_logger().info("Recieving PointCloud2 message")
        
        # Read points from pointcloud2 messages
        points = []
        for point in read_points(data, field_names=("x" , "y" , "z", "r" , "g" , "b"), skip_nans= True):
            points.append(points)
            
        # Convert PointCloud into a numpy for processing
        points_array = np.array(points, dtype = np.float32)
        
        # Print Number of points recieved 
        self.get_logger().info(f"Point Cloud recieved with {points_array.shape[0]} points")
        
        
        # Commented this. This is an experiment
        ''' To visualize Point cloud as 2d Projection'''

        if points_array > 0 : 
            image = self.visualize_point_cloud(points_array)
            cv2.imshow("Point Cloud" , image) 
            cv2.waitKey() 
            
    def visualize_point_cloud (self, points_array):
        """ Visualize the PCL as 2d Projection"""
        # Project x, y, z into 2D
        
        x = points_array[:, 0]
        z = points_array[:, 2]       
        img = np.zeros((480, 640), dtype = np.uint8)
        
        # Normalize and scale the Points into image dimensions
        x_norm = ((x - np.min(x)) / (np.max(x) - np.min(x)) * (img.shape[1] -1)).astype(int) 
        z_norm = ((z - np.min(z)) / (np.max(z) - np.min(z)) * (img.shape[0] -1)).astype(int)

        # Populate 2D image
        
        img[z_norm, x_norm] = 255
        
        return img


def main (args = None) : 
    rclpy.init(args = args)
    intel_subscriber = IntelSubscriber()
    try:
        rclpy.spin(intel_subscriber)
    except KeyboardInterrupt:
        intel_subscriber.get_logger().info("Node interrupted")
    finally: 
        intel_subscriber.destroy_node()
        rclpy.shutdown()




if __name__ == "__main__":
    main()