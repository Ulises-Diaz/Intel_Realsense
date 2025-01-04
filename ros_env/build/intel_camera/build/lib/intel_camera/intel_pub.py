import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np 

class IntelPublisher (Node) :
    def __init__(self): 
        super().__init__("Intel_pulisher") #Name displayed when there's mesages. U can change it
        self.intel_publisher_rgb = self.create_publisher(Image, "rgb_frame", 10) # (type_msg, topic name (u can change it), qeue list) 
    
        timer_period = 0.05
        self.br_rgb = CvBridge() #convert numpy into sensor msgs
        
        
        try : 
            self.pipe = rs.pipeline() #to recieve the frames from the camera
            self.cfg = rs.config ()
            self.cfg.enable_stream (rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipe.start(self.cfg)
            self.timer = self.create_timer(timer_period, self.timer_callback) #timer to send information

        except Exception as e : 
            print(e)
            self.get_logger().error("Intel RealSense is not connected")
            
    def timer_callback (self) : 
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        
        self.intel_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(color_image))
        self.get_logger().info("Publishing RGB frame")
            
            
            
def main (args = None):
    rclpy.init(args = None)
    intel_publisher = IntelPublisher()
    rclpy.spin(intel_publisher)
    intel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__" : 
    main()