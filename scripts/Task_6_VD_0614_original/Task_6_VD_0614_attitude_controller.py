#!/usr/bin/env python

'''
# Team ID:          0614
# Theme:            Vitarana Drone
# Author List:      Utkarsh Shahdeo,Pranav Sharma,Ashish Chouhan,Aman Srivastava
# Filename:         Task_6_VD_0614_attitude_controller.py
# Functions:        init,imu_callback,drone_command_callback,pid,main
# Global variables: None
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):

        '''
    Purpose:
    ---
    Function to initialise various variables of Edrone class
          
    Input Arguments:
    ---
    None

    Returns:
    ---
    None
    
    Example call:
    ---
   Called when an object of Edrone class is made
    '''



        rospy.init_node('attitude_controller')  # initializing ros node with name attitude_controller

      
        # List to store current orientation of drone in quaternion format
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        self.lastTime=0.0 # To store the last time

        # List to store current orientation of drone in euler angles format
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # List to store roll,pitch and yaw setpoint of drone in 1000 to 2000 range
        self.setpoint_cmd = [0.0, 0.0, 0.0]
        self.setpoint_throttle = 0.0  # Setpoint for throttle

        # List to store roll,pitch and yaw setpoint of drone in euler angles
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        
        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]
        
        self.Kp = [3.28, 3.61, 201.0]
        self.Ki = [0, 0, 0]
        self.Kd = [4.63, 4.57, 0.5]

             
        # Initialising lists to store errors
        
        self.current_error=[0, 0, 0]       # To store current error in [roll, pitch, yaw] format
        self.previous_error=[0, 0, 0]      # To store previous error in [roll, pitch, yaw] format
    
        # Storing max and min values for pwm

        self.max_values = [1024, 1024, 1024, 1024] # Max limit corresponding to [prop1, prop2, prop3, prop4]
        self.min_values = [0, 0, 0, 0]             # Min limit corresponding to [prop1, prop2, prop3, prop4]
        
        # Proportional,Integral and Derivative values in [roll, pitch, yaw] format

        self.P = [0, 0, 0]
        self.I = [0, 0, 0]   
        self.D = [0, 0, 0]     

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_frequency = 16  # in hertz

        # PUBLISHERS
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        
        # SUBSCRIBERS
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        
    
    def imu_callback(self, msg):

        
        '''
    Purpose:
    ---
    Function to get current orientation of drone using imu
          
    Input Arguments:
    ---
    `msg` :  [ Object ]
        Stores current orientation of drone in quarternion format
    
    Returns:
    ---
    None
    
    Example call:
    ---
    Called whenever we get the data from the publisher
        '''

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w
        
         
       
    def drone_command_callback(self, msg):

        '''
    Purpose:
    ---
    Function to get target orientation of drone
          
    Input Arguments:
    ---
    `msg` :  [ Object ]
        Stores target orientation of drone in 1000 to 2000 range
    
    Returns:
    ---
    None
    
    Example call:
    ---
    Called whenever we get the data from the publisher
        '''



        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_throttle = msg.rcThrottle
 
  
   

     
    def pid(self):

        '''
    Purpose:
    ---
    Function to calculate pid values and give pwm values    
 
          
    Input Arguments:
    ---
    `msg` :  [ Object ]
        Stores target orientation of drone in quarternion format
    
    Returns:
    ---
    None
    
    Example call:
    ---
    Called whenever we get the data from the publisher
        '''




        now=time.time() # Number of seconds passed since epoch

              
        # Converting drone current orientation from quaternion to euler angles
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0],         self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Converting radians to degrees
        self.drone_orientation_euler[0]=math.degrees(self.drone_orientation_euler[0])
        self.drone_orientation_euler[1]=math.degrees(self.drone_orientation_euler[1])
        self.drone_orientation_euler[2]=math.degrees(self.drone_orientation_euler[2])

       

        
       

        # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30
 
        elapsed_time=now-self.lastTime # Time elapsed since pid function has been called

        if(elapsed_time>=0.05):

            # Calculating error in orientation
            self.current_error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0]
            self.current_error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1]
            self.current_error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]

                               
          
            # PID equations for roll 
        
            self.P[0]=self.Kp[0]*self.current_error[0]
            self.I[0]=self.I[0] + self.Ki[0]*self.current_error[0]*elapsed_time
            self.D[0]=self.Kd[0]*(self.current_error[0]-self.previous_error[0])/elapsed_time

            self.out_roll=self.P[0] + self.I[0] + self.D[0]
            
            
            # PID equations for pitch 
        
            self.P[1]=self.Kp[1]*self.current_error[1]
            self.I[1]=self.I[1] + self.Ki[1]*self.current_error[1]*elapsed_time
            self.D[1]=self.Kd[1]*(self.current_error[1]-self.previous_error[1])/elapsed_time

            self.out_pitch=self.P[1] + self.I[1] + self.D[1]
            

            # PID equations for yaw 
        
            self.P[2]=self.Kp[2]*self.current_error[2]
            self.I[2]=self.I[2] + self.Ki[2]*self.current_error[2]*elapsed_time
            self.D[2]=self.Kd[2]*(self.current_error[2]-self.previous_error[2])/elapsed_time

            self.out_yaw=self.P[2] + self.I[2] + self.D[2]

            
                
            # Saving current error as previous error

            self.previous_error[0]=self.current_error[0]
            self.previous_error[1]=self.current_error[1]
            self.previous_error[2]=self.current_error[2]

            self.lastTime=now
        

            # Giving throttle to each motor in 1000-2000 format

            motor1 = self.setpoint_throttle + self.out_roll - self.out_pitch - self.out_yaw
            motor2 = self.setpoint_throttle - self.out_roll - self.out_pitch + self.out_yaw
            motor3 = self.setpoint_throttle - self.out_roll + self.out_pitch - self.out_yaw
            motor4 = self.setpoint_throttle + self.out_roll + self.out_pitch + self.out_yaw
            

        
            # Converting the range of 1000-2000 to 0-1024 for pwm format
        
            self.pwm_cmd.prop1 = 1.024*motor1 - 1024
            self.pwm_cmd.prop2 = 1.024*motor2 - 1024
            self.pwm_cmd.prop3 = 1.024*motor3 - 1024
            self.pwm_cmd.prop4 = 1.024*motor4 - 1024

            # Bounding the values of pwm between 0 and 1024
    
            if(self.pwm_cmd.prop1>1024):
                self.pwm_cmd.prop1=1024
            elif(self.pwm_cmd.prop1<0):
                self.pwm_cmd.prop1=0
            else:
                self.pwm_cmd.prop1=self.pwm_cmd.prop1

            if(self.pwm_cmd.prop2>1024):
                self.pwm_cmd.prop2=1024
            elif(self.pwm_cmd.prop2<0):
                self.pwm_cmd.prop2=0
            else:
                self.pwm_cmd.prop2=self.pwm_cmd.prop2

            if(self.pwm_cmd.prop3>1024):
                self.pwm_cmd.prop3=1024
            elif(self.pwm_cmd.prop3<0):
                self.pwm_cmd.prop3=0
            else:
                self.pwm_cmd.prop3=self.pwm_cmd.prop3

            if(self.pwm_cmd.prop4>1024):
                self.pwm_cmd.prop4=1024
            elif(self.pwm_cmd.prop4<0):
                self.pwm_cmd.prop4=0
            else:
                self.pwm_cmd.prop4=self.pwm_cmd.prop4


       
            # Publishing the propellers pwm
            self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':

    '''
    Purpose:
    ---
    Main function,creates object of class Edrone and calls the pid function continously at a fixed frequency
          
    Input Arguments:
    ---
    None

    Returns:
    ---
    None
    
    Example call:
    ---
   Called at the start of executing the script
        '''



    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_frequency) 
    rospy.loginfo("started")
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
