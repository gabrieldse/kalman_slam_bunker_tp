#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
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

class Graph(Node):
    def __init__(self):

        super().__init__('Graph')
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, 1)
        self.sub_odom_estim = self.create_subscription(Pose2D, '/odom_estim', self.cb_estim, 1)
        self.timer=self.create_timer(1, self.update_graph)
        self.position=Pose2D()
        

        self.fig = plt.figure(1)
        self.fig.clf()
        axis=self.fig.gca()
        axis.set_xlim(-10.0,10.0)
        axis.set_ylim(-5.0,5.0)
        self.t_pos_x=[]
        self.t_pos_y=[]
        
        self.t_estim_x=[]
        self.t_estim_y=[]

        plt.draw()
        plt.pause(0.001)


    def cb_odom(self,msg):
        self.position.x=msg.pose.pose.position.x
        self.position.y=msg.pose.pose.position.y
        roll,pitch,yaw=euler_from_quaternion(msg.pose.pose.orientation)
        self.position.theta=yaw
        self.t_pos_x.append(self.position.x)
        self.t_pos_y.append(self.position.y)
        # plt.scatter(self.position.x,self.position.y,s=5,color = 'red')

    def cb_estim(self,msg):
        self.get_logger().info("estim")
        self.t_estim_x.append(msg.x)
        self.t_estim_y.append(msg.y)
        # plt.scatter(msg.x,msg.y,s=5,color = 'blue')
        
    def update_graph(self):
        plt.clf()
        plt.scatter(self.t_pos_x,self.t_pos_y,s=5,color = 'red')
        plt.scatter(self.t_estim_x,self.t_estim_y,s=5,color = 'blue')
        plt.grid()
        plt.draw()

        plt.pause(0.0001)



def main(args=None):
    rclpy.init(args=args)
    node=Graph()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()