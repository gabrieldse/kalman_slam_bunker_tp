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


class Kalman(Node):

    def __init__(self):

        super().__init__('Kalman')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        # use_sim_time = self.get_parameter('use_sim_time').value
        # self.get_logger().info(f"use_sim_time: {use_sim_time}")

        self.get_logger().info("Kalman start")                   

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, 1)
        self.sub_tag = self.create_subscription(ChannelFloat32,"/tag",self.cb_tag,1)
        self.pub_cmd_vel =self.create_publisher(Twist,"/cmd_vel",1)
        self.pub_estim=self.create_publisher(Pose2D,"/odom_estim",1)

        self.dt=0.1
        self.timer=self.create_timer(self.dt, self.commande)
        
        self.sim_time=0.0
        self.cmd=Twist()
        self.t=0.0
        self.cmd.linear.x=0.0

        self.tag_recept=False
        self.tag=ChannelFloat32()
        self.tag.values= [0.0,0.0,0.0,0.0,0.0,0.0]
        self.position=Pose2D()
        self.estim=Pose2D()

        # Variable of xt
        self.epsilon = 0.0
        self.epsilon_array = []
        self.mu_moyen = 0.0
        self.epsilon_moyen = 0.0


    #envoie une commande toutes les 50ms
    def commande(self):
        # self.get_logger().info("cmd") 
        self.t=self.t+self.dt
        if self.t>0.5:
            self.cmd.linear.x=0.5
        if self.t>10.0:
            self.cmd.linear.x=0.0

            # self.angular.z=1.0
        self.pub_cmd_vel.publish(self.cmd)
        
        # Estimation
        self.estim.x=self.estim.x+self.dt*self.cmd.linear.x

        ### Setting up the actionning state equation xt----------------------------------------------------------
        # Epsilon
        self.epsilon = self.position.x - self.estim.x
        self.epsilon_array.append(self.epsilon)

        self.mu_moyen = np.mean(self.epsilon_array)
        self.epsilon_moyen = np.var(self.epsilon_array)

        # # self.get_logger().info(" ")
        # # self.get_logger().info(f"Mu moyen: {self.mu_moyen}")
        # # self.get_logger().info(f"Ecart Type: {self.epsilon_moyen}")
        # # self.get_logger().info(" ")

        ### Setting up the percetpion state equation zt----------------------------------------------------------
        self.get_logger().info(f"Mu moyen: {self.mu_moyen}")

        # pour envoyer la position estimée au noeud graph
        self.pub_estim.publish(self.estim)
        #self.get_logger().info(f"epsilon: {self.epsilon}")

        # pour  détecter si probleme de synchro temporelle
        # tout va bien si delta reste constant
        delta=self.sim_time-self.t
        # self.get_logger().info(f"delta {delta:6.1f}")

        

    # récupération des informations de tag
    # dans self.tag.values on trouve [tag_id,dist,tag_id2,dist2,0,0]

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
        self.sim_time=msg.header.stamp.sec+1e-9*msg.header.stamp.nanosec
        
        # gazebo pas a pas
        # ros2 service call /step_world gazebo_msgs/srv/StepWorld "{num_steps: 1}"



def main(args=None):
    rclpy.init(args=args)
    node=Kalman()
    # node.set_parameters('use_sim_time', True)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()


