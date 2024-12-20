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
        self.state_gazebo=Pose2D()
        self.state_estim=Pose2D()
        self.state=Pose2D()

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
        self.sigma = np.identity(3) # Init of the State Covariance matrix 
        self.Rt = [ [0.0196, 0, 0],[0, 0.0196, 0],[0, 0,np.pi/180] ] ## todo correct 1 degree of error in variance
        self.G =  np.zeros([3,3])
        self.Qt = 0.000208 # Camera variance, measured on the camera experiment
        self.Ht = np.zeros([1,3])
        self.Kt = np.zeros([3,1])
        self.d_state_estim = 0
        self.z = 0
    
        

    #envoie une commande toutes les 50ms
    def commande(self):
        # self.get_logger().info("cmd") 
        self.t=self.t+self.dt
        if self.t>0.5:
            self.v = 0.5

            ### Calculate the VARIANCE of the ODOM (state) measurement --------------------
            # self.error_on_pos = self.state_gazebo.x - self.state.x
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
        
        self.cmd.linear.x = self.v*np.cos(self.state.theta)
        self.get_logger().info(f" ******* [ DEBUG ******** ]  self.cmd.linear.x = {self.cmd.linear.x}; self.v = {self.v}, np.cos(self.state.theta) = {np.cos(self.state.theta)}")
        self.cmd.linear.y = self.v*np.sin(self.state.theta)
       
        

        ##### KALMAN CALCULATION #####

        ### EQ 1 - State Estimation Update
        self.state_estim.x = self.state.x + self.dt*self.cmd.linear.x
        self.get_logger().info(f" ******* [ DEBUG ]  self.state.x = {self.state.x}; self.state_estim.x = {self.state_estim.x}")
        self.state_estim.y = self.state.y + self.dt*self.cmd.linear.y
        self.state_estim.theta = self.state.theta + self.dt * self.cmd.angular.z 
        

        ### Section 2 - Uncertainty State (Jacobian matrix )
        
        self.G = np.array([[1, 0, -self.v * self.dt * np.sin(self.state.theta)],
                           [0, 1, self.v * self.dt * np.cos(self.state.theta)],
                           [0, 0, 1]])
        
        # delta x and delta y (distance along x, and y between the robot and the obstacle)
        self.dx = self.obsx - self.state_estim.x
        self.dy = self.obsy - self.state_estim.y
        self.d = np.sqrt(self.dx**2 + self.dy**2) # Distance between where we think the robot is and the obstacle
        
        self.get_logger().info(f" ******* [ DEBUG ]  D_estimated = {self.d}; robot estimX = {self.state.x}, robot estimY = {self.state.y}, obstacle X = {self.obsx}, obstacle Y = {self.obsy}")
                
        # Actual EQ 2
        self.sigma_estime = self.G @ self.sigma @ np.transpose(self.G) + self.Rt
        
        ### Section 3 - Kalman GAIN calculation
        
        self.Ht = np.array([[self.dx/self.d, self.dy/self.d, 0]]) # correction np.array ([[a, b, c]]), double [[]] to make a 1x3 array
        
        # EQ 3
        # self.get_logger().info(f" ******* [ DEBUG ] sigma_estima = {np.shape(self.sigma_estime)}; Ht_transp = {np.shape(np.transpose(self.Ht))}, Ht = {np.shape(self.Ht)}, Qt = {np.shape(self.Qt)}" )
        self.Kt = self.sigma_estime @ np.transpose(self.Ht) @ np.linalg.inv( self.Ht @ self.sigma_estime @ np.transpose(self.Ht)  + self.Qt ) 
  
        
        ### Section 4 
        
        # Measurement Model
        self.d_state_estim = np.array([self.d]) # are both supose to be zero at the second coordinate ?
        self.z = np.array([self.tag.values[1]])# measure of the appril tag
        
        """
        If no measure from the camera is obtained, do not use (obstacle distance - 0), 8 for the first case. As an update 
        """
        if self.z != 0:
            update = self.Kt @ ( self.z - self.d_state_estim ) # ERROR
        else:
            update = np.zeros([3])
        
        # Kalman update
        self.state.x = self.state_estim.x + update[0]
        self.state.y = self.state_estim.y + update[1]
        self.state.theta = self.state_estim.theta + update[2]
        
        self.get_logger().info(f" ******* [ DEBUG ] d estimated from self.estim   {self.d_state_estim}")
        self.get_logger().info(f" ******* [ DEBUG ] d obtained from camera {self.z}")
        
        ### EQ 5 - Update the covariance matrix
        self.sigma = (np.identity(3)-self.Kt @ self.Ht) @ self.sigma_estime

        ### PUBLISH ESTIMATION
        self.pub_estim.publish(self.state)

        ### PUBLISH COMMANDS
        delta=self.sim_time-self.t
        self.pub_cmd_vel.publish(self.cmd)

    # récupération des informations de tag
    # dans self.tag.values on trouve [tag_id,dist,tag_id2,dist2,0,0]

    def cb_tag(self, msg):
        self.tag=msg
        self.tag_recept=True


    #récupération de la position réelle du robot dans le simulateur
    # et reconversion dans self.state_gazebo pour avoir un message 2D x,y,theta
    def cb_odom(self,msg):

        self.state_gazebo.x=msg.pose.pose.position.x
        self.state_gazebo.y=msg.pose.pose.position.y
        roll,pitch,yaw=euler_from_quaternion(msg.pose.pose.orientation)
        self.state_gazebo.theta=yaw
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