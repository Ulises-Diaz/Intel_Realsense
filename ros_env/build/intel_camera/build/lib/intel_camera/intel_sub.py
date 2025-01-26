import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from cv_bridge import CvBridge
import cv2
import numpy as np


class IntelSubscriber(Node):
    def __init__(self):
        super().__init__("Intel_subscriber")
        self.subscription_rgb = self.create_subscription(Image, "rgb_frame", self.rgb_frame_callback, 10)
        self.subscription_pcl = self.create_subscription(PointCloud2, "Point_cloud", self.point_cloud_callback, 10)
        self.br_rgb = CvBridge()

    def rgb_frame_callback(self, data):
        """Callback for receiving RGB frames."""
        self.get_logger().info("Receiving RGB frame")
        try:
            current_frame = self.br_rgb.imgmsg_to_cv2(data, desired_encoding="bgr8")
            cv2.imshow("RGB", current_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing RGB frame: {e}")

    def point_cloud_callback(self, data):
        """Callback for receiving PointCloud2 messages."""
        self.get_logger().info("Receiving PointCloud2 message")
        try:
            # Read points from PointCloud2 messages
            points = [
                [point[0], point[1], point[2], point[3], point[4], point[5]]  # x, y, z, r, g, b
                for point in read_points(data, field_names=("x", "y", "z", "r", "g", "b"), skip_nans=True)
            ]

            # Convert to a NumPy array
            points_array = np.array(points, dtype=np.float32)

            if points_array.size > 0:
                self.get_logger().info(f"PointCloud2 received with {points_array.shape[0]} points")
                image = self.visualize_point_cloud(points_array)
                cv2.imshow("Point Cloud", image)
                cv2.waitKey(1)
            else:
                self.get_logger().warning("Received empty PointCloud2 message.")
        except Exception as e:
            self.get_logger().error(f"Error processing PointCloud2: {e}")

    def visualize_point_cloud(self, points_array):
        """Visualize the PointCloud2 as a 2D projection."""
        try:
            # Down-sample points for faster visualization
            points_array = points_array[::10]  # Process every 10th point

            # Project x, z into 2D
            x = points_array[:, 0]
            z = points_array[:, 2]
            img = np.zeros((480, 640), dtype=np.uint8)

            # Normalize and scale points into image dimensions
            x_norm = np.clip(((x - np.min(x)) / (np.max(x) - np.min(x)) * (img.shape[1] - 1)).astype(int), 0, img.shape[1] - 1)
            z_norm = np.clip(((z - np.min(z)) / (np.max(z) - np.min(z)) * (img.shape[0] - 1)).astype(int), 0, img.shape[0] - 1)

            # Populate the 2D image
            img[z_norm, x_norm] = 255

            return img
        except Exception as e:
            self.get_logger().error(f"Error visualizing PointCloud2: {e}")
            return np.zeros((480, 640), dtype=np.uint8)


def main(args=None):
    rclpy.init(args=args)
    intel_subscriber = IntelSubscriber()
    try:
        rclpy.spin(intel_subscriber)
    except KeyboardInterrupt:
        intel_subscriber.get_logger().info("Node interrupted.")
    finally:
        intel_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
