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
        self.pub_estim=self.create_publisher(Pose2D,"/odom_ekf",1)

        self.dt=0.1
        self.timer=self.create_timer(self.dt, self.commande)
        
        self.sim_time=0.0
        self.cmd=Twist()
        self.t=0.0
        self.v = 0.0 # commande ne vitesse du robot
        self.d = 0.0 #distance with the cam
        self.dx = 0.0 #distance cam_robot along x axis
        self.dy = 0.0 #distance cam_robot along y axis

        #Obstacle location
        self.obsx = 8.0
        self.obsy = 0.0

        self.tag_recept=False
        self.tag=ChannelFloat32()
        self.tag.values= [0.0,0.0,0.0,0.0,0.0,0.0]
        self.position=Pose2D()
        self.estim=Pose2D()

        # Estimation of epsilon of xt
        self.error_on_pos = 0.0
        self.error_on_pos_array = []
        self.mu_moyen = 0.0
        self.variance_epsilon = 0.0 #Rt

        # Variable of delta of xt
        self.cam_dist_array = []
        self.cam_dist_mean = 0.0
        self.cam_dist_var = 0.0

        #Matrix definition
        self.sigma = [[1, 0, 0],[0, 1, 0],[0, 0, 1]] # Init of the State Covariance matrix 
        self.Rt = [ [0.0196, 0, 0],[0, 0.0196, 0],[0, 0,np.pi/180] ] ## todo correct 1 degree of error in variance
        self.G =  np.zeros([3,3])
        self.Qt = 0.000208 # Camera variance, measured on the camera experiment
        self.Ht = np.zeros([1,3])
        self.Kt = np.zeros([3,2])
        self.h_mu_estim = 0
        self.z = 0
    
        

    #envoie une commande toutes les 50ms
    def commande(self):
        # self.get_logger().info("cmd") 
        self.t=self.t+self.dt
        if self.t>0.5:
            self.v = 0.5

            ### Calculate the VARIANCE of the ODOM (state) measurement --------------------
            # self.error_on_pos = self.position.x - self.estim.x
            # self.error_on_pos_array.append(self.error_on_pos)
            # self.mu_moyen = np.mean(self.error_on_pos_array)
            # self.variance_epsilon = np.var(self.error_on_pos_array)
            # self.get_logger().info(f"self.error_on_pos: {self.error_on_pos}")
            # self.get_logger().info(f"----------------------------------")

        if self.t>11.0:
            self.v = 0

            ### Calculate the VARIANCE of the camera measurement --------------------
            # self.cam_dist_array.append(self.tag.values[1])
            # self.cam_dist_mean = np.mean(self.cam_dist_array)
            # self.cam_dist_var = np.var(self.cam_dist_array)
            # self.get_logger().info(f"self.cam_dist_var: {self.cam_dist_var}")

        #Speed parameters
        self.cmd.linear.x = self.v*np.cos(self.estim.theta)
        self.cmd.linear.y = self.v*np.sin(self.estim.theta)
       
        

        ##### KALMAN CALCULATION #####

        ### EQ 1 - State Estimation Update
        self.estim.x = self.estim.x + self.dt*self.cmd.linear.x
        self.estim.y = self.estim.y + self.dt*self.cmd.linear.y
        self.estim.theta = self.estim.theta + self.dt * self.cmd.angular.z 
        

        ### Section 2 - Uncertainty State (Jacobian matrix )
        
        self.G = np.array([[1, 0, -self.v * self.dt * np.sin(self.estim.theta)],
                           [0, 1, self.v * self.dt * np.cos(self.estim.theta)],
                           [0, 0, 1]])
        
        # delta x and delta y (distance along x, and y between the robot and the obstacle)
        self.dx = self.obsx - self.estim.x
        self.dy = self.obsy - self.estim.y
        self.d = np.sqrt(self.dx**2 + self.dy**2) # Distance between where we think the robot is and the obstacle
        
        self.get_logger().info(f" ******* [ DEBUG ]  D_estimated = {self.d}; robot estimX = {self.estim.x}, robot estimY = {self.estim.y}, obstable X = {self.obsx}")
                
        # Actual EQ 2
        self.sigma_estime = self.G @ self.sigma @ np.transpose(self.G) + self.Rt
        
        ### Section 3 - Kalman GAIN calculation
        
        self.Ht = np.array([[self.dx/self.d, self.dy/self.d, 0]]) # correction np.array ([[a, b, c]]), double [[]] to make a 1x3 array
        
        # EQ 3
        # self.get_logger().info(f" ******* [ DEBUG ] sigma_estima = {np.shape(self.sigma_estime)}; Ht_transp = {np.shape(np.transpose(self.Ht))}, Ht = {np.shape(self.Ht)}, Qt = {np.shape(self.Qt)}" )
        self.Kt = self.sigma_estime @ np.transpose(self.Ht) @ np.linalg.inv( self.Ht @ self.sigma_estime @ np.transpose(self.Ht)  + self.Qt ) 
  
        
        ### Section 4 
        
        # Measurement Model
        self.h_mu_estim = np.array([self.d]) # are both supose to be zero at the second coordinate ?
        self.z = np.array([self.tag.values[1]])# measure of the appril tag
        update = self.Kt @ ( self.z - self.h_mu_estim ) # ERROR
        
        # Kalman update
        self.estim.x = self.estim.x + update[0]
        self.estim.y = self.estim.y + update[1]
        self.estim.theta = self.estim.theta + update[2]
        
        self.get_logger().info(f"h_mu_estim {self.h_mu_estim}")
        self.get_logger().info(f"zt {self.z}")
        
        ### EQ 5 - Update the covariance matrix
        self.sigma = (np.identity(3)-self.Kt @ self.Ht) @ self.sigma_estime

        ### PUBLISH ESTIMATION
        self.pub_estim.publish(self.estim)

        ### PUBLISH COMMANDS
        delta=self.sim_time-self.t
        self.pub_cmd_vel.publish(self.cmd)

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