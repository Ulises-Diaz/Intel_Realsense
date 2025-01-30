import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class IntelSubscriber(Node) : 
    def __init__ (self) : 
        super().__init__("Intel_subscriber")
        self.subscription_rgb = self.create_subscription(Image, "rgb_frame", self.rgb_frame_callback, 10)
        self.subscription_bg_removed = self.create_subscription(Image, "bg_removed_frame", self.bg_removed_callback, 10)
        
        
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

def main (args = None) : 
    rclpy.init(args = args)
    intel_subscriber = IntelSubscriber()
    rclpy.spin(intel_subscriber)
    intel_subscriber.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__":
    main()