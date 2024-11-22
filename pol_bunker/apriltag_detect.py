#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo,ChannelFloat32 # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import cv2 # OpenCV library
from math import sqrt


#https://pypi.org/project/pyapriltags/
#pip install pyapriltags
import pyapriltags


class ImageSubscriber(Node):

    def __init__(self):

        super().__init__('image_subscriber')
        self.get_logger().info("Apriltag")                   

        self.sub_image = self.create_subscription(Image, '/camera_1/image_raw', self.cb_image, 1)
        self.sub_info = self.create_subscription(CameraInfo, '/camera_1/camera_info', self.cb_info, 1)
        self.pub_tag = self.create_publisher(ChannelFloat32,"/tag",1)
        self.param=False
        self.br = CvBridge()
        # self.params=[640.5098521801531, 640.5098521801531, 640.5, 360.5]

        self.at_detector = pyapriltags.Detector(searchpath=['apriltags'],
                        families='tag36h11',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)
        self.tag=ChannelFloat32()

        
        
   
    def cb_image(self, data):
        # self.get_logger().info("Recup Camera image")                   

        image = self.br.imgmsg_to_cv2(data)
        grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # affiche l'image capturée
        # cv2.imshow('Result', image)
        # cv2.waitKey(1)
        
        

        if self.param:
            self.tag.values= [0.0,0.0,0.0,0.0,0.0]
            tags = self.at_detector.detect(grayimg, estimate_tag_pose=True, camera_params=self.params, tag_size=0.5)
            nbtag=0
            while len(tags)>0:
                
                v1=tags.pop()
                d=self.dist(v1)
                x=v1.pose_t[0]
                y=v1.pose_t[1]
                z=v1.pose_t[2]
                # print(v1.center)
                # print(v1.corners)
                # print(v1.pose_t)
                # print(f"tag {v1.tag_id} - distance :{d}")
                self.tag.values[2*nbtag]=v1.tag_id
                self.tag.values[2*nbtag+1]=d
                nbtag+=1
            if nbtag>0:
                self.pub_tag.publish(self.tag)


    def cb_info(self,msg):
        self.param=True
        # self.get_logger().info("Recup Camera info")                   

        #https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        self.fx=msg.k[0]
        self.fy=msg.k[4]
        self.cx=msg.k[2]
        self.cy=msg.k[5]
        self.params=[self.fx,self.fy,self.cx,self.cy]
        # print(self.params)

    def dist(self,v1):
        x=v1.pose_t[0]
        y=v1.pose_t[1]
        z=v1.pose_t[2]
        return sqrt(x*x+y*y+z*z)










def main(args=None):
    rclpy.init(args=args)
    node=ImageSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()


