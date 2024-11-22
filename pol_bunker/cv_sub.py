#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

class ImageSubscriber(Node):

  def __init__(self):
    super().__init__('image_subscriber')
    self.get_logger().info('init subscriber')
   
    self.subscription = self.create_subscription(
      Image, '/color/image_raw', self.listener_callback, 1)    
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')
     # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    # output=current_frame.copy()

    # gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    # gray=cv2.blur(gray, (3, 3))


    cv2.imshow("current_frame", current_frame)

    cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node=ImageSubscriber()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()

