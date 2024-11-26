#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo,ChannelFloat32 # Image is the message type
from geometry_msgs.msg import Twist,Pose2D
from nav_msgs.msg import Odometry
import numpy as np
from math import sqrt,asin,atan2
import matplotlib.pyplot as plt

def euler_from_quaternion(q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


class Kalman(Node):

    def __init__(self):

        super().__init__('Kalman')
        self.get_logger().info("Kalman start")  
        self.dt=0.1                 

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, 1)
        self.sub_tag = self.create_subscription(ChannelFloat32,"/tag",self.cb_tag,1)
        self.pub_cmd_vel =self.create_publisher(Twist,"/cmd_vel",1)
        self.timer=self.create_timer(self.dt, self.commande)
        
        self.cmd=Twist()
        self.t=0.0
        self.cmd.linear.x=0.0

        self.tag_recept=False
        self.tag=ChannelFloat32()
        self.tag.values= [0.0,0.0,0.0,0.0,0.0]
        self.position=Pose2D()
        self.fig = plt.figure(1)
        self.fig.clf()
        axis=self.fig.gca()
        axis.set_xlim(-1.0,10.0)
        axis.set_ylim(-3.0,3.0)
      
        plt.draw()
        plt.grid()
        plt.pause(0.001)

    #envoie une commande toutes les 50ms
    def commande(self):
        # self.get_logger().info("cmd") 
        self.t=self.t+0.1
        if self.t>0.5:
            self.cmd.linear.x=0.5
        if self.t>5.0:
            self.cmd.linear.x=0.0
            # self.angular.z=1.0
            self.get_logger().info(f"Final X position:{self.position.x}")
        self.pub_cmd_vel.publish(self.cmd)
    
    #récupération des informations de tag
    #dans self.tag.values on trouve [tag_id,dist,tag_id2,dist2,0,0]
    def cb_tag(self, msg):
        self.tag=msg
        self.tag_recept=True


    #récupération de la position réelle du robot dans le simulateur
    # et reconversion dans self.position pour avoir un message 2D x,y,theta
    def cb_odom(self,msg):
        self.position.x=msg.pose.pose.position.x
        self.position.y=msg.pose.pose.position.y
        roll,pitch,yaw=euler_from_quaternion(msg.pose.pose.orientation)
        self.position.theta=yaw
        plt.scatter(self.position.x,self.position.y,s=5,color = 'red')
        # plt.show(block=False)
        plt.draw()
        plt.pause(0.0001)
        


def main(args=None):
    rclpy.init(args=args)
    node=Kalman()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()