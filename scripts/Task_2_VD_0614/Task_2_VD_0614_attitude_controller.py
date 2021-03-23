#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /pid_tuning_altitude
        /pitch_error            /pid_tuning_pitch
        /yaw_error              /pid_tuning_roll
        /edrone/pwm             /edrone/imu/data
                                /edrone/drone_command
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
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
        rospy.init_node('attitude_controller')  # initializing ros node with name attitude_control

        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        self.lastTime=0.0 # To store the last time

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [0.0, 0.0, 0.0]
        self.setpoint_throttle = 0.0  # Setpoint for throttle

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # Declaring roll_error,pitch_error and yaw_error of message type Float32 and initializing values

        #self.roll_error = Float32()
        #self.pitch_error = Float32()
        #self.yaw_error = Float32()

        #self.roll_error.data = 0.0
        #self.pitch_error.data = 0.0
        #self.yaw_error.data = 0.0
        
              

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
        self.sample_frequency = 30  # in hertz

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------
        #self.roll_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        #self.pitch_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        #self.yaw_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)
        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw, /pid_tuning_altitude
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        #rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        #rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        #rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        
        # ------------------------------------------------------------------------------------------------------------

    # Function to get current orientation of drone using imu
      
    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w
        
    # Function to get current target orientation of drone    
       
    def drone_command_callback(self, msg):

        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_throttle = msg.rcThrottle
 
  
    #def roll_set_pid(self, roll):
        #self.Kp[0] = roll.Kp * 0.01  # This is just for an example. You can change the ratio/fraction value accordingly
        #self.Ki[0] = roll.Ki * 0.001
        #self.Kd[0] = roll.Kd * 0.01

    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.01  
        self.Ki[1] = pitch.Ki * 0.001
        self.Kd[1] = pitch.Kd * 0.01
     
    
    #def yaw_set_pid(self, yaw):
        #self.Kp[2] = yaw.Kp*0.1   
        #self.Ki[2] = yaw.Ki*0.1 
        #self.Kd[2] = yaw.Kd*0.1 

    


    # Function to calculate pid values and give pwm values    
 
    def pid(self):

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

            # Storing the current error which will be published later 
            #self.roll_error.data = self.current_error[0]
            #self.pitch_error.data = self.current_error[1]
            #self.yaw_error.data = self.current_error[2]

            # Publishing the error
            #self.roll_pub.publish(self.roll_error)
            #self.pitch_pub.publish(self.pitch_error)
            #self.yaw_pub.publish(self.yaw_error)
        
                      
          
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

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_frequency)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    rospy.loginfo("started")
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
