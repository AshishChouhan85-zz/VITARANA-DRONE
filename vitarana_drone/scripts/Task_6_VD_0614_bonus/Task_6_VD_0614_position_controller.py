#!/usr/bin/env python

'''
# Team ID:          0614
# Theme:            Vitarana Drone
# Author List:      Utkarsh Shahdeo,Pranav Sharma,Ashish Chouhan,Aman Srivastava
# Filename:         Task_6_VD_0614_position_controller.py
# Functions:        init,gps_callback,hold_box,top_callback,marker_coord,pid_global,pid_local,pid_search,move_global,move_local,
                    local_escape,search_marker,Sort,main
# Global variables: None
'''

# Importing the required libraries

from vitarana_drone.msg import edrone_cmd
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from vitarana_drone.srv import *
import math
import rospy
import time
from sensor_msgs.msg import LaserScan
import csv
import numpy as np
import pandas as pd


class Edrone():
    
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


        rospy.init_node('position_controller') # Initializing ros node with name position_controller

        # List to append main setpoints
        self.setpoints =[]
        # List to append setpoints after which we need to search for marker
        self.target_setpoints=[] 
        # List to append setpoints for avoiding obstacle
        self.setpoints_local=[]  
        # List to append setpoints for searching for the marker
        self.search_setpoints=[]
        
        # Flag to give permission to append coordinates in local setpoints
        self.change=1  
        # Flag to give permission to append coordinates to escape the obstacle
        self.escape_change=1
        # Flag to give permission to append coordinates in search setpoints
        self.search_change=1              

        # To store the last time     
        self.lastTime=0.0  
        # To store current time      
        self.now=0.0             

        # To store value of current index number of main setpoints
        self.r=0  
        # To manipulate index of target setpoints   
        self.i=0     

        # Flag to allow searching or quit searching
        self.search=0    
        # To store previous previous index of global list                  
        self.search_index=0
        # Flag which tells whether the marker is detcted or not to give coordinates to move towards the marker                
        self.detected=0                   
        # Focal length of camera used
        self.focal_length =238.350718905 
        # To store latitude that will lead to marker  
        self.global_x=0   
        # To store longitude that will lead to marker                 
        self.global_y=0      
        # Flag which becomes 1 when x distance of drone from centre of marker becomes less than 0.15 metres               
        self.lat=0 
        # Flag which becomes 1 when y distance of drone from centre of marker becomes less than 0.2 metres                     
        self.longt=0  
        # Flag which store latitude for which x distance becomes less than 0.15 metres                     
        self.final_x=0  
        # Flag which store longitude for which y distance becomes less than 0.2 metres                   
        self.final_y=0 
        # Flag which gives setpoint to decrease altitude of drone after it reaches the threshold box above the marker                    
        self.reached=0  
        # So that only 1 setpoint is given by flag self.reached
        self.wait=0                        
        
       

        # To store distance between two setpoints
        self.path_dist=0
        # To store whether absolute value of latitude is more or longitude    
        self.path="null" 
        # Sign of latitude in case of destination coordinate minus current coordinate  
        self.sign_x=1 
        # Sign of longitude in case of destination coordinate minus current coordinate 
        self.sign_y=1 
        # To store whether obstacle is in front,back,left or right      
        self.obstacle="null" 


        # To store whether gripper will be activated or not
        self.value = True   
        # So that only one coordinate is given to escape the obstacle 
        self.count=0  
      

        # To end the script
        self.end=0           
       
        # To store distance of obstacle from back
        self.back=0
        # To store distance of obstacle from front 
        self.front=0 
        # To store distance of obstacle from left
        self.left=0
        # To store distance of obstacle from right 
        self.right=0 

        # To store distance from obstacle in latitude
        self.obstacle_dist_x=0 
        # To store distance from obstacle in longitude    
        self.obstacle_dist_y=0   

       



        # List for storing current [latitude,longitude,altitude]
        self.current_coord = [0.0,0.0,0.0]

        
        # Declaring drone_orientation of message type edrone_cmd and initialising values

        self.drone_orientation = edrone_cmd()

               

        # initial setting of Kp, Kd and ki for [latitude, longitude, altitude]

        self.Kp = [1750000, 1730000,  110] 
        self.Ki = [0.0, 0.0, 5.0]
        self.Kd = [13700000, 13700000, 500] 

        # initial setting of Kp, Kd and ki for [latitude, longitude, altitude] while box is held by drone

        self.Kp_hold = [1750000, 1730000,  300] 
        self.Ki_hold = [0.0, 0.0, 5.0]  
        self.Kd_hold = [14500000, 13700000, 800] 

       
        # Initialising lists to store errors

        # To store current error in [latitude,longitude,altitude] format
        self.current_error=[0, 0, 0]    
        # To store previous error of global setpoints in [latitude,longitude,altitude] format   
        self.previous_error=[0, 0, 0] 
        # To store previous error of local setpoints in [latitude,longitude,altitude] format        
        self.previous_error_local=[0, 0, 0]


        # Storing max and min values for pwm

        # Max limit corresponding to [rcRoll,rcPitch,rcYaw,rcThrottle]
        self.max_values = [2000,2000,2000,2000] 
        # Min limit corresponding to [rcRoll,rcPitch,rcYaw,rcThrottle]
        self.min_values = [1000,1000,1000,1000] 
        

        # Proportional,Integral and Derivative values in [latitude,longitude,altitude] format

        self.P = [0, 0, 0]
        self.I = [0, 0, 0]   
        self.D = [0, 0, 0]  

        # Proportional,Integral and Derivative values in [latitude,longitude,altitude] format while holding box

        self.P_hold = [0, 0, 0]
        self.I_hold = [0, 0, 0]   
        self.D_hold = [0, 0, 0]    

        

        # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation   step time is 50 ms
        self.sample_frequency = 60  # in hertz

        # PUBLISHERS

        self.rc_pub = rospy.Publisher('/drone_command',edrone_cmd,queue_size=1)
        
        # SUBSCRIBERS

        rospy.Subscriber('/edrone/gps',NavSatFix,self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top',LaserScan,self.top_callback)
        rospy.Subscriber("pixel_coord",String,self.marker_coord)

        # SERVICES

        # Creating connection to service
        self.gp_srv = rospy.ServiceProxy('/edrone/activate_gripper',Gripper)
        # Waiting for service to be running
        self.gp_srv.wait_for_service() 

       


    def gps_callback(self,msg):

        '''
    Purpose:
    ---
    Callback function to get gps coordinates of the drone.
    
    Input Arguments:
    ---
    `msg` :  [ Object ]
        Stores current latitude,longitude and altitude of drone
    
    Returns:
    ---
    None
    
    Example call:
    ---
    Called whenever we get the data from the publihser
        '''
       
        self.current_coord[0] = msg.latitude
        self.current_coord[1] = msg.longitude
        self.current_coord[2] = msg.altitude  



    def hold_box(self,value):

        '''
    Purpose:
    ---
    Function to activate or deactivate the gripper
    
    Input Arguments:
    ---
    `value` :  [ bool ]
        Stores True or False according to whether we have to activate or deactivate the gripper.
        True means to activate the gripper,similarly False is used.
    
    Returns:
    ---
    'resp.result' : [ bool ]
        Stores True or False on the basis of whether gripper has grabed the box or not
    
    Example call:
    ---
    self.hold_box(True)
        '''

        req=GripperRequest()                  
        
        req.activate_gripper=value
               
        resp=self.gp_srv(req)  
               
          
        return resp.result



    def top_callback(self,msg):

        
        '''
    Purpose:
    ---
    Callback function to get the obstacle distance in front,back,left or right side of the drone
    
    Input Arguments:
    ---
    `msg` :  [ Object ]
        Stores distance of obstacle in front,back,left or right side of the drone

    Returns:
    ---
    None

    Example call:
    ---
    Called whenever we get the data from the publisher
        '''

        # Flags to tell direction of obstacle detected are refreshed 
        self.back=0
        self.front=0
        self.left=0
        self.right=0

        left=msg.ranges[0]
        front=msg.ranges[1]
        right=msg.ranges[2]
        back=msg.ranges[3]


        # This condition is used so that data from distance sensors is used when index 3 of coordinate list is 1,number of coordinates is not exceeded and marker is not being searched by drone
        if(self.r!=len(self.setpoints) and self.setpoints[self.r][3]==1 and self.search==0): 
            
            # If obstacle is detected in back 
            if(back<=8 and back>=0.4): 
                self.back=1
                        
                self.obstacle_dist_x=back 
        
             # If obstacle is detected in back
            if(front<=9 and front>=0.4): 
                self.front=1
                        
                self.obstacle_dist_x=front
                
            # If obstacle is detected in left
            if(left<=8 and left>=0.4): 
                self.left=1
                       
                self.obstacle_dist_y=left

            # If obstacle is detected in right        
            if(right<=8 and right>=0.4 ): 
                self.right=1
                       
                self.obstacle_dist_y=right

        # To turn sensors off when drone is searching for marker
        if(self.search==1): 
            self.back=0
            self.front=0
            self.left=0
            self.right=0 

        
    def marker_coord(self,msg):
           
         
        '''
    Purpose:
    ---
    Callback function to get pixel coordinates of centre of marker and to 
    convert them into distance (in metres) from drone and finally converts them into latitude and longitude
    form to be given to drone as a setpoint  
    
    Input Arguments:
    ---
    `msg` :  [ Object ]
        Stores X and Y coordinates of centre of detected marker

    Returns:
    ---
    None

    Example call:
    ---
    Called whenever we get the data from the publisher
        '''
        
        # If any marker is detected then only calculate the coordinartes of marker 
        if(msg.data!="NO MARKER"):
  
            txt=msg.data
            txt_lst=txt.split(",")
            X=txt_lst[0]
            Y=txt_lst[1]

            # Getting x coordinate of centre pixel of marker
            X=int(X)  
            # Getting y coordinate of centre pixel of marker       
            Y=int(Y)         
            
            # Distance upto which drone will hover from given global setpoint of building terrace, to detect marker
            Z_m=16.75      
            # Getting x distance from centre of marker to drone in metres  
            dist_x=(X*Z_m)/self.focal_length 
            # Getting y distance from centre of marker to drone in metres     
            dist_y=(Y*Z_m)/self.focal_length     

            # Removing constant error from x distance
            dist_x=abs(dist_x-14.065193393445954) 
            # Removing constant error from y distance 
            dist_y=abs(dist_y-14.336017175437686) 

            # Giving proper sign for correct latitude and longitude calculation 
            if(Y>200):                            
                sign_y=1
            else:
                sign_y=-1
            
            if(X>200):
                sign_x=1
            else:
                sign_x=-1   
                     
            # Getting approx coordinates to centre of marker  
            self.global_x=self.current_coord[0]+(dist_x/110692.0702932625)*sign_x
            self.global_y=self.current_coord[1]+(dist_y/105292.0089353767)*sign_y 
                     
                       
            # Getting coordinates of centre of marker which are inside the threshold of 0.2 metres
            if((dist_x<=0.15) and (self.detected==1)): 
                self.final_x=self.current_coord[0]
                self.lat=1
            else:
                self.lat=0

            if((dist_y<=0.2) and (self.detected==1)):
                self.final_y=self.current_coord[1]
                self.longt=1
            else:
                self.longt=0

            # If we get any coordinate such that drone is inside the threshold then we save it
            if(self.lat==1):
                self.global_x=self.final_x

            if(self.longt==1):
                self.global_y=self.final_y 
    
            # When drone is inside the threshold box,then value of some flags are updated 
            if((self.lat+self.longt)==2 and dist_x<=0.15 and dist_y<=0.2):
                # Stop moving towards marker as we have reached it
                self.detected=0 
                # Tell drone that it has reached the marker and now decrease its altitude  
                self.reached=1     
               
                
        
    
    def pid_global(self,index_no,elapsed_time):

        '''
    Purpose:
    ---
    Function to calculate pid values to move to final setpoints,converts them into values used by drone and publishing them

    Input Arguments:
    ---
    `index_no` :  [ int ]
        Stores current index number of setpoints list 
    'elapsed_time' : [ float ]
        Stores time elapsed in calculating pid values

    Returns:
    ---
    None

    Example call:
    ---
    self.pid_global(3,0.02)
        '''




        # Calculating error in orientation

        self.current_error[0] = self.setpoints[index_no][0] - self.current_coord[0]
        self.current_error[1] = self.setpoints[index_no][1] - self.current_coord[1]
        self.current_error[2] = self.setpoints[index_no][2] - self.current_coord[2]

        # To give value to activate/deactivate gripper,hold stores True/False if box is attached to gripper or not
        hold=self.hold_box(self.value) 
        
       
        # If hold is false then use pid values for drone only                

        if(hold==False):

            # PID equations for lattitude 
              
            self.P[0]=self.Kp[0]*self.current_error[0]
            self.I[0]=self.I[0] + self.Ki[0]*self.current_error[0]*elapsed_time
            self.D[0]=self.Kd[0]*(self.current_error[0]-self.previous_error[0])/elapsed_time
           
            self.out_lat=self.P[0] + self.I[0] + self.D[0]

            # PID equations for longitude 
              
            self.P[1]=self.Kp[1]*self.current_error[1]
            self.I[1]=self.I[1] + self.Ki[1]*self.current_error[1]*elapsed_time
            self.D[1]=self.Kd[1]*(self.current_error[1]-self.previous_error[1])/elapsed_time
            
            self.out_long=self.P[1] + self.I[1] + self.D[1]

            # PID equations for altitude 
              
            self.P[2]=self.Kp[2]*self.current_error[2]

            if(self.current_error[2]<0.028 and self.current_error[2]>-0.028):
                self.I[2]=self.I[2] + self.Ki[2]*self.current_error[2]*elapsed_time
            self.D[2]=self.Kd[2]*(self.current_error[2]-self.previous_error[2])/elapsed_time

            self.out_alt=self.P[2] + self.I[2] + self.D[2]
                    
        # If hold is true then use pid values for drone plus Package                 
                     
        if(hold==True):

            # PID equations for lattitude in holding position
              
            self.P_hold[0]=self.Kp_hold[0]*self.current_error[0]
            self.I_hold[0]=self.I_hold[0] + self.Ki_hold[0]*self.current_error[0]*elapsed_time
            self.D_hold[0]=self.Kd_hold[0]*(self.current_error[0]-self.previous_error[0])/elapsed_time
            
            self.out_lat=self.P_hold[0] + self.I_hold[0] + self.D_hold[0]

            # PID equations for longitude in holding position
              
            self.P_hold[1]=self.Kp_hold[1]*self.current_error[1]
            self.I_hold[1]=self.I_hold[1] + self.Ki_hold[1]*self.current_error[1]*elapsed_time
            self.D_hold[1]=self.Kd_hold[1]*(self.current_error[1]-self.previous_error[1])/elapsed_time
            
            self.out_long=self.P_hold[1] + self.I_hold[1] + self.D_hold[1]

            # PID equations for altitude in holding position
              
            self.P_hold[2]=self.Kp_hold[2]*self.current_error[2]

            if(self.current_error[2]<0.11 and self.current_error[2]>-0.11):
                self.I_hold[2]=self.I_hold[2] + self.Ki_hold[2]*self.current_error[2]*elapsed_time
            self.D_hold[2]=self.Kd_hold[2]*(self.current_error[2]-self.previous_error[2])/elapsed_time
                    
            if(self.I_hold[2]<-30): 
                self.I_hold[2]=-30
                      
            if(self.I_hold[2]>30): 
                self.I_hold[2]=30

            self.out_alt=self.P_hold[2] + self.I_hold[2] + self.D_hold[2]
                    
                    
                  
                                 
        # Saving current error as previous error
            
        self.previous_error[0]=self.current_error[0]  
        self.previous_error[1]=self.current_error[1]
        self.previous_error[2]=self.current_error[2]
        self.lastTime=self.now 

        # Giving required orientation values 
                               
        self.drone_orientation.rcRoll = 1500 + self.out_long
        self.drone_orientation.rcPitch = 1500 + self.out_lat
        self.drone_orientation.rcYaw = 1500
        self.drone_orientation.rcThrottle = 1500 +self.out_alt
                
        # Bounding the values of required orientation between 1000 and 2000

        if(self.drone_orientation.rcThrottle>2000):
            self.drone_orientation.rcThrottle=2000
        elif(self.drone_orientation.rcThrottle<1000):
            self.drone_orientation.rcThrottle=1000
        else:
            self.drone_orientation.rcThrottle=self.drone_orientation.rcThrottle

        if(self.drone_orientation.rcRoll>2000):
            self.drone_orientation.rcRoll=2000
        elif(self.drone_orientation.rcRoll<1000):
            self.drone_orientation.rcRoll=1000
        else:
            self.drone_orientation.rcRoll=self.drone_orientation.rcRoll

        if(self.drone_orientation.rcPitch>2000):
            self.drone_orientation.rcPitch=2000
        elif(self.drone_orientation.rcPitch<1000):
            self.drone_orientation.rcPitch=1000
        else:
            self.drone_orientation.rcPitch=self.drone_orientation.rcPitch

        if(self.drone_orientation.rcYaw>2000):
            self.drone_orientation.rcYaw=2000
        elif(self.drone_orientation.rcYaw<1000):
            self.drone_orientation.rcYaw=1000
        else:
            self.drone_orientation.rcYaw=self.drone_orientation.rcYaw
                
        # Publishing the orientations
        self.rc_pub.publish(self.drone_orientation)

 

    def pid_local(self,elapsed_time):     


        '''
    Purpose:
    ---
    Function to calculate pid values for setpoints to avoid obstacles,converts them into values used by drone and     publishing them

    Input Arguments:
    ---
    'elapsed_time' : [ float ]
        Stores time elapsed in calculating pid values

    Returns:
    ---
    None

    Example call:
    ---
    self.pid_local(0.02)
        '''


        # Calculating error in orientation
                
        self.current_error[0] = self.setpoints_local[0][0] - self.current_coord[0]
        self.current_error[1] = self.setpoints_local[0][1] - self.current_coord[1]
        self.current_error[2] = self.setpoints_local[0][2] - self.current_coord[2]

        # To give value to activate/deactivate gripper,hold stores True/False if box is attached to gripper or not
        hold=self.hold_box(self.value) 

        # If hold is false then use pid values for drone only                
        if(hold==False):

            # PID equations for lattitude 
              
            self.P[0]=self.Kp[0]*self.current_error[0]
            self.I[0]=self.I[0] + self.Ki[0]*self.current_error[0]*elapsed_time
            self.D[0]=self.Kd[0]*(self.current_error[0]-self.previous_error_local[0])/elapsed_time
          
            self.out_lat=self.P[0] + self.I[0] + self.D[0]

            # PID equations for longitude 
              
	    self.P[1]=self.Kp[1]*self.current_error[1]
            self.I[1]=self.I[1] + self.Ki[1]*self.current_error[1]*elapsed_time
            self.D[1]=self.Kd[1]*(self.current_error[1]-self.previous_error_local[1])/elapsed_time
            
            self.out_long=self.P[1] + self.I[1] + self.D[1]

            # PID equations for altitude 
              
            self.P[2]=self.Kp[2]*self.current_error[2]

            if(self.current_error[2]<0.028 and self.current_error[2]>-0.028):
                self.I[2]=self.I[2] + self.Ki[2]*self.current_error[2]*elapsed_time
            self.D[2]=self.Kd[2]*(self.current_error[2]-self.previous_error_local[2])/elapsed_time
            
            self.out_alt=self.P[2] + self.I[2] + self.D[2]
                    
            # If hold is true then use pid values for drone plus Package                 
                     
        if(hold==True):

            # PID equations for lattitude in holding position
              
            self.P_hold[0]=self.Kp_hold[0]*self.current_error[0]
            self.I_hold[0]=self.I_hold[0] + self.Ki_hold[0]*self.current_error[0]*elapsed_time
            self.D_hold[0]=self.Kd_hold[0]*(self.current_error[0]-self.previous_error_local[0])/elapsed_time
            
            self.out_lat=self.P_hold[0] + self.I_hold[0] + self.D_hold[0]

            # PID equations for longitude in holding position
              
            self.P_hold[1]=self.Kp_hold[1]*self.current_error[1]
            self.I_hold[1]=self.I_hold[1] + self.Ki_hold[1]*self.current_error[1]*elapsed_time
            self.D_hold[1]=self.Kd_hold[1]*(self.current_error[1]-self.previous_error_local[1])/elapsed_time
            
            self.out_long=self.P_hold[1] + self.I_hold[1] + self.D_hold[1]

            # PID equations for altitude in holding position
              
            self.P_hold[2]=self.Kp_hold[2]*self.current_error[2]

            if(self.current_error[2]<0.11 and self.current_error[2]>-0.11):
                self.I_hold[2]=self.I_hold[2] + self.Ki_hold[2]*self.current_error[2]*elapsed_time
            self.D_hold[2]=self.Kd_hold[2]*(self.current_error[2]-self.previous_error_local[2])/elapsed_time
                    
            if(self.I_hold[2]<-30): 
                self.I_hold[2]=-30
                      
            if(self.I_hold[2]>30): 
                self.I_hold[2]=30

            self.out_alt=self.P_hold[2] + self.I_hold[2] + self.D_hold[2]
                    
                    
                  
                                 
        # Saving current error as previous error
            
        self.previous_error_local[0]=self.current_error[0]  
        self.previous_error_local[1]=self.current_error[1]
        self.previous_error_local[2]=self.current_error[2]
        self.lastTime=self.now # Updating the value of last time

        # Giving required orientation values 
                               
        self.drone_orientation.rcRoll = 1500 + self.out_long
        self.drone_orientation.rcPitch = 1500 + self.out_lat
        self.drone_orientation.rcYaw = 1500
        self.drone_orientation.rcThrottle = 1500 +self.out_alt
                
        # Bounding the values of required orientation between 1000 and 2000

        if(self.drone_orientation.rcThrottle>2000):
            self.drone_orientation.rcThrottle=2000
        elif(self.drone_orientation.rcThrottle<1000):
            self.drone_orientation.rcThrottle=1000
        else:
            self.drone_orientation.rcThrottle=self.drone_orientation.rcThrottle
           
        if(self.drone_orientation.rcRoll>2000):
            self.drone_orientation.rcRoll=2000
        elif(self.drone_orientation.rcRoll<1000):
            self.drone_orientation.rcRoll=1000
        else:
            self.drone_orientation.rcRoll=self.drone_orientation.rcRoll

        if(self.drone_orientation.rcPitch>2000):
            self.drone_orientation.rcPitch=2000
        elif(self.drone_orientation.rcPitch<1000):
            self.drone_orientation.rcPitch=1000
        else:
            self.drone_orientation.rcPitch=self.drone_orientation.rcPitch

        if(self.drone_orientation.rcYaw>2000):
            self.drone_orientation.rcYaw=2000
        elif(self.drone_orientation.rcYaw<1000):
            self.drone_orientation.rcYaw=1000
        else:
            self.drone_orientation.rcYaw=self.drone_orientation.rcYaw
                
                
        # Publishing the orientations
        self.rc_pub.publish(self.drone_orientation)



    def pid_search(self,elapsed_time):

        
        '''
    Purpose:
    ---
    Function to calculate pid values for setpoints to search for markers,converts them into values used by drone and     publishing them

    Input Arguments:
    ---
    'elapsed_time' : [ float ]
        Stores time elapsed in calculating pid values

    Returns:
    ---
    None

    Example call:
    ---
    self.pid_search(0.02)
        '''



        # Calculating error in orientation
                
        self.current_error[0] = self.search_setpoints[0][0] - self.current_coord[0]
        self.current_error[1] = self.search_setpoints[0][1] - self.current_coord[1]
        self.current_error[2] = self.search_setpoints[0][2] - self.current_coord[2]

        # To give value to activate/deactivate gripper,hold stores True/False if box is attached to gripper or not
        hold=self.hold_box(self.value) 
                
        # If hold is false then use pid values for drone only                
        if(hold==False):

            # PID equations for lattitude 
              
            self.P[0]=self.Kp[0]*self.current_error[0]
            self.I[0]=self.I[0] + self.Ki[0]*self.current_error[0]*elapsed_time
            self.D[0]=self.Kd[0]*(self.current_error[0]-self.previous_error_local[0])/elapsed_time
                    
                   

            self.out_lat=self.P[0] + self.I[0] + self.D[0]

            # PID equations for longitude 
              
	    self.P[1]=self.Kp[1]*self.current_error[1]
            self.I[1]=self.I[1] + self.Ki[1]*self.current_error[1]*elapsed_time
            self.D[1]=self.Kd[1]*(self.current_error[1]-self.previous_error_local[1])/elapsed_time
            
            self.out_long=self.P[1] + self.I[1] + self.D[1]

            # PID equations for altitude 
              
            self.P[2]=self.Kp[2]*self.current_error[2]

            if(self.current_error[2]<0.028 and self.current_error[2]>-0.028):
                self.I[2]=self.I[2] + self.Ki[2]*self.current_error[2]*elapsed_time
            self.D[2]=self.Kd[2]*(self.current_error[2]-self.previous_error_local[2])/elapsed_time

                    
            
            self.out_alt=self.P[2] + self.I[2] + self.D[2]
                    
            # If hold is true then use pid values for drone plus Package                 
                     
        if(hold==True):

            # PID equations for lattitude in holding position
              
            self.P_hold[0]=self.Kp_hold[0]*self.current_error[0]
            self.I_hold[0]=self.I_hold[0] + self.Ki_hold[0]*self.current_error[0]*elapsed_time
            self.D_hold[0]=self.Kd_hold[0]*(self.current_error[0]-self.previous_error_local[0])/elapsed_time
            
            self.out_lat=self.P_hold[0] + self.I_hold[0] + self.D_hold[0]

            # PID equations for longitude in holding position
              
            self.P_hold[1]=self.Kp_hold[1]*self.current_error[1]
            self.I_hold[1]=self.I_hold[1] + self.Ki_hold[1]*self.current_error[1]*elapsed_time
            self.D_hold[1]=self.Kd_hold[1]*(self.current_error[1]-self.previous_error_local[1])/elapsed_time
            
            self.out_long=self.P_hold[1] + self.I_hold[1] + self.D_hold[1]

            # PID equations for altitude in holding position
              
            self.P_hold[2]=self.Kp_hold[2]*self.current_error[2]

            if(self.current_error[2]<0.11 and self.current_error[2]>-0.11):
                self.I_hold[2]=self.I_hold[2] + self.Ki_hold[2]*self.current_error[2]*elapsed_time
            self.D_hold[2]=self.Kd_hold[2]*(self.current_error[2]-self.previous_error_local[2])/elapsed_time
                    
            if(self.I_hold[2]<-30): 
                self.I_hold[2]=-30
                      
            if(self.I_hold[2]>30): 
                self.I_hold[2]=30

            self.out_alt=self.P_hold[2] + self.I_hold[2] + self.D_hold[2]
                    
                    
                  
                                 
        # Saving current error as previous error
            
        self.previous_error_local[0]=self.current_error[0]  
        self.previous_error_local[1]=self.current_error[1]
        self.previous_error_local[2]=self.current_error[2]
        self.lastTime=self.now # Updating the value of last time

        # Giving required orientation values 
                               
        self.drone_orientation.rcRoll = 1500 + self.out_long
        self.drone_orientation.rcPitch = 1500 + self.out_lat
        self.drone_orientation.rcYaw = 1500
        self.drone_orientation.rcThrottle = 1500 +self.out_alt
                
        # Bounding the values of required orientation between 1000 and 2000

        if(self.drone_orientation.rcThrottle>2000):
            self.drone_orientation.rcThrottle=2000
        elif(self.drone_orientation.rcThrottle<1000):
            self.drone_orientation.rcThrottle=1000
        else:
            self.drone_orientation.rcThrottle=self.drone_orientation.rcThrottle
           
        if(self.drone_orientation.rcRoll>2000):
            self.drone_orientation.rcRoll=2000
        elif(self.drone_orientation.rcRoll<1000):
            self.drone_orientation.rcRoll=1000
        else:
            self.drone_orientation.rcRoll=self.drone_orientation.rcRoll

        if(self.drone_orientation.rcPitch>2000):
            self.drone_orientation.rcPitch=2000
        elif(self.drone_orientation.rcPitch<1000):
            self.drone_orientation.rcPitch=1000
        else:
            self.drone_orientation.rcPitch=self.drone_orientation.rcPitch

        if(self.drone_orientation.rcYaw>2000):
            self.drone_orientation.rcYaw=2000
        elif(self.drone_orientation.rcYaw<1000):
            self.drone_orientation.rcYaw=1000
        else:
            self.drone_orientation.rcYaw=self.drone_orientation.rcYaw
                
                
        # Publishing the orientations
        self.rc_pub.publish(self.drone_orientation)
    

    def move_global(self,index_no):

        '''
    Purpose:
    ---
    Function to update the index of setpoints list once a setpoint is inside the defined threshold box.This
    function also gives drone the command to search for marker when the time comes and also compares whether latitude or 
    longitude is greater which will be used to avoid obstacle if drone encounters any.

    Input Arguments:
    ---
    'index_no' : [ int ]
        Stores index number of current setpoint of the list setpoints

    Returns:
    ---
   'index_no' : [ int ]
        Stores index number of current setpoint of the list setpoints

    Example call:
    ---
    self.move_global(2)
        '''

        # Passing the value of current index number so that there is no scope of error as sometimes None also gets passed
        index_no=self.r 

        # Getting the current time
        self.now=time.time() 
        
        # Time elapsed 
        elapsed_time=self.now - self.lastTime 
        
        
        # Sample time
        if(elapsed_time>=0.001): 
            

            if(index_no!=len(self.setpoints) and index_no<len(self.setpoints)):

                # PID values are calculated and published to drone in proper format
                self.pid_global(index_no,elapsed_time)
                

                # This condition is for setpoints for which drone needs high accuracy
                if(self.setpoints[index_no][3]==0):
                
                # Setting the tolerance for lattitude,longitude and altitude
                # If the current coordinates is within the bounded range then lat/longt/alt is set to 1,else it is set to 0
           
                    if(((self.setpoints[index_no][0]-0.000001717)<self.current_coord[0]) and ((self.setpoints[index_no][0]+0.000001717)>self.current_coord[0])):
                        lat=1
                    else:
                        lat=0 
           
                    if(((self.setpoints[index_no][1]-0.0000017487)<self.current_coord[1]) and ((self.setpoints[index_no][1]+0.0000017487)>self.current_coord[1])):
                        longt=1
                    else:
                        longt=0  

                    if(((self.setpoints[index_no][2]-0.05)<self.current_coord[2]) and ((self.setpoints[index_no][2]+0.05)>self.current_coord[2])):
                        alt=1
                    else:
                        alt=0 

                else:

                    if(((self.setpoints[index_no][0]-0.000004000)<self.current_coord[0]) and ((self.setpoints[index_no][0]+0.000004000)>self.current_coord[0])):
                        lat=1
                    else:
                        lat=0 
           
                    if(((self.setpoints[index_no][1]-0.000004000)<self.current_coord[1]) and ((self.setpoints[index_no][1]+0.000004000)>self.current_coord[1])):
                        longt=1
                    else:
                        longt=0  

                    if(((self.setpoints[index_no][2]-0.05)<self.current_coord[2]) and ((self.setpoints[index_no][2]+0.05)>self.current_coord[2])):
                        alt=1
                    else:
                        alt=0 


                              
                    
                # If any setpoint is reached then code prforms the following tasks
                if(lat+longt+alt==3):
                    
                    # Instruction to drop or hold the box is given in setpoint
                    if(self.setpoints[index_no][4]=="DROP"):
                        self.value=False
                    else:
                        self.value=True

                    # If drone reaches a setpoint where we have to search for marker
                    if(self.target_setpoints[self.i]==self.setpoints[index_no]):

                        # Start searching for marker
                        self.search=1
                        # Current index number of setpoints list is stored
                        self.search_index=index_no
                        # These flags are turned to 0 as drone is not inside the threshold box of marker
                        self.lat=0
                        self.longt=0
                        # This is used to give only one coordinate for landing the drone on marker
                        self.wait=0
                        # To start detecting the marker
                        self.detected=1
                        # Used to tell drone that it is exactly above the marker and it can descend now
                        self.reached=0
                        # Updating index number of target_setpoints list
                        self.i=self.i+1


                        if(self.i>=len(self.target_setpoints)):
                            self.i=len(self.target_setpoints)-1
                    else:
                        self.search=0
                       
                       
                    index_no=index_no+1


                    if(index_no<=(len(self.setpoints)-1) and self.search!=1):
                       
                        # Calculating total latitude distance between current setpoint and destination setpoint in metres
                        x=(abs(self.setpoints[index_no][0]-self.setpoints[index_no-1][0]))*110692.0702932625 
                        # Calculating total longitude distance between current setpoint and destination setpoint in metres
                        y=(abs(self.setpoints[index_no][1]-self.setpoints[index_no-1][1]))*105292.0089353767 
                        # Calculating distance between current setpoint and destination setpoint in metres  
                        self.path_dist=(x**2+y**2)**0.5


                        # To store whether latitude is greater or longitude(To be used in obstacle avoidance)
                        if(x>y):
                            self.path="X"
                        else:
                            self.path="Y"

                        # To store whether drone is going in positive or negative direction(To be used in obstacle avoidance)
                        if((self.setpoints[index_no][0]-self.setpoints[index_no-1][0])>0):
                            self.sign_x=1
                        else:
                            self.sign_x=-1         

                        if((self.setpoints[index_no][1]-self.setpoints[index_no-1][1])>0):
                            self.sign_y=1
                        else:
                            self.sign_y=-1

                        
                
                        
                    
                else:
                    index_no=index_no

                # To shutdown the drone at the end

                if(index_no==len(self.setpoints)):
                    self.drone_orientation.rcRoll = 1500
                    self.drone_orientation.rcPitch = 1500
                    self.drone_orientation.rcYaw = 1500
                    self.drone_orientation.rcThrottle = 1000
                    self.rc_pub.publish(self.drone_orientation)
                    self.end=1
                    
                
               

            self.r=index_no # Storing the current index number

            

            return index_no


    def move_local(self):

        
        '''
    Purpose:
    ---
    Function to calculate the setpoints for avoiding the obstacle.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None   

    Example call:
    ---
    self.move_local()
        '''


        self.count=1
        # Value of change is 1 when it is beginning,required local setpoint is reached or when during escape drone encounters an obstacle
        if(self.change==1): 
            # List is cleared so that index number remains 0 
            self.setpoints_local=[] 
            
            # If latitude is greater
            if(self.path=="X"):           
                if(self.back==1 and self.sign_x!=1): 
                    # For maintaining a fixed distance from obstacle
                    lati=self.current_coord[0] + ((5-self.obstacle_dist_x)/110692.0702932625)
                    # For moving along obstacle
                    longi=self.current_coord[1]+(5.5/105292.0089353767) 
                    # Storing direction of obstacle(later used for escaping the obstacle)
                    self.obstacle="BACK"
                    self.setpoints_local.append([lati,longi,self.current_coord[2]])
                    

                if(self.front==1 and self.sign_x!=-1): 
                    lati=self.current_coord[0] - ((5-self.obstacle_dist_x)/110692.0702932625)
                    longi=self.current_coord[1]+(5.5/105292.0089353767)    
                    self.obstacle="FRONT"  
                    self.setpoints_local.append([lati,longi,self.current_coord[2]])      
                   

                if(self.left==1):
                    lati=self.current_coord[0]+(4.5/110692.0702932625)*self.sign_x
                    longi=self.current_coord[1] + ((4.5-self.obstacle_dist_y)/105292.0089353767)
                    self.obstacle="LEFT"
                    self.setpoints_local.append([lati,longi,self.current_coord[2]])
                     

                if(self.right==1):
                    lati=self.current_coord[0]+(4.5/110692.0702932625)*self.sign_x
                    longi=self.current_coord[1] - ((4.5-self.obstacle_dist_y)/105292.0089353767)
                    self.obstacle="RIGHT"
                    self.setpoints_local.append([lati,longi,self.current_coord[2]])



            if(self.path=="Y"): 
          
                if(self.back==1):
                    # For maintaining a fixed distance from obstacle 
                    lati=self.current_coord[0] + ((5-self.obstacle_dist_x)/110692.0702932625) 
                    # For moving along obstacle
                    longi=self.current_coord[1]+(5.5/105292.0089353767)*self.sign_y 
                    self.obstacle="BACK"
                    self.setpoints_local.append([lati,longi,self.current_coord[2]])
                    

                if(self.front==1): 
                    lati=self.current_coord[0] - ((5-self.obstacle_dist_x)/110692.0702932625)
                    longi=self.current_coord[1]+(5.5/105292.0089353767)*self.sign_y    
                    self.obstacle="FRONT"  
                    self.setpoints_local.append([lati,longi,self.current_coord[2]])      
                   

                if(self.left==1 and self.sign_y!=1):
                    lati=self.current_coord[0]+(4.5/110692.0702932625)
                    longi=self.current_coord[1] + ((4.5-self.obstacle_dist_y)/105292.0089353767)
                    self.obstacle="LEFT"
                    self.setpoints_local.append([lati,longi,self.current_coord[2]])
                     

                if(self.right==1 and self.sign_y!=-1):
                    lati=self.current_coord[0]+(4.5/110692.0702932625)*self.sign_x
                    longi=self.current_coord[1] - ((4.5-self.obstacle_dist_y)/105292.0089353767)
                    self.obstacle="RIGHT"
                    self.setpoints_local.append([lati,longi,self.current_coord[2]])






            
            
        
        
        self.now=time.time() # Getting the current time

        

        elapsed_time=self.now - self.lastTime # Time elapsed 
        
        
        
        if(elapsed_time>=0.001 and len(self.setpoints_local)==1): # Sample time
            self.pid_local(elapsed_time)


            # Setting the tolerance for lattitude,longitude and altitude
            # If the current coordinates is within the bounded range then lat/longt/alt is set to 1,else it is set to 0
            
            if(((self.setpoints_local[0][0]-0.000002717)<self.current_coord[0]) and ((self.setpoints_local[0][0]+0.000002717)>self.current_coord[0])):
                lat=1
            else:
                lat=0 
           
            if(((self.setpoints_local[0][1]-0.0000027487)<self.current_coord[1]) and ((self.setpoints_local[0][1]+0.0000027487)>self.current_coord[1])):
                longt=1
            else:
                longt=0  

            if(((self.setpoints_local[0][2]-0.05)<self.current_coord[2]) and ((self.setpoints_local[0][2]+0.05)>self.current_coord[2])):
                alt=1
            else:
                alt=0 
            
                       

            if(lat+longt+alt==3):
                 
                self.setpoints_local=[]
                self.change=1
                
            else:
                self.change=0


    def local_escape(self): 
             
        '''
    Purpose:
    ---
    Function to escape the obstacle.This function helps to reach the local setpoint even when obstacle has gone but local     setpoint is not reached
           

    Input Arguments:
    ---
    None

    Returns:
    ---
    None   

    Example call:
    ---
    self.local_escape()
        '''
        

        

        self.now=time.time() # Getting the current time

        

        elapsed_time=self.now - self.lastTime # Time elapsed 
        
        
        
        if(elapsed_time>=0.001 and len(self.setpoints_local)==1): # Sample time
            self.pid_local(elapsed_time)


            # Setting the tolerance for lattitude,longitude and altitude
            # If the current coordinates is within the bounded range then lat/longt/alt is set to 1,else it is set to 0
            
            if(((self.setpoints_local[0][0]-0.000002717)<self.current_coord[0]) and ((self.setpoints_local[0][0]+0.000002717)>self.current_coord[0])):
                lat=1
            else:
                lat=0 
           
            if(((self.setpoints_local[0][1]-0.0000027487)<self.current_coord[1]) and ((self.setpoints_local[0][1]+0.0000027487)>self.current_coord[1])):
                longt=1
            else:
                longt=0  

            if(((self.setpoints_local[0][2]-0.05)<self.current_coord[2]) and ((self.setpoints_local[0][2]+0.05)>self.current_coord[2])):
                alt=1
            else:
                alt=0 
            
                       

            if(lat+longt+alt==3):
                self.setpoints_local=[]
                self.change=1
                self.escape_change=1

            else:
                self.change=1
                self.escape_change=0

            # Coordinates are given here so that to escape the obstacle

            if(self.escape_change==1 and self.count==1):

                # If latitude is greater than longitude
                if(self.path=="X"):
                    self.count=self.count+1
 
                    # If obstacle was in front or back
                    if(self.obstacle=="FRONT" or self.obstacle=="BACK"):
                        lati=self.current_coord[0]+(7/110692.0702932625)*self.sign_x
                        longi=self.current_coord[1]+(2/105292.0089353767)
                        alti=self.current_coord[2]
                        self.setpoints_local.append([lati,longi,alti])

                    # If obstacle was in left
                    if(self.obstacle=="LEFT"):
                        lati=self.current_coord[0]+(1/110692.0702932625)*self.sign_x
                        longi=self.current_coord[1]
                        alti=self.current_coord[2]
                        self.setpoints_local.append([lati,longi,alti])
 

           
    def search_marker(self):

        '''
    Purpose:
    ---
    Function to search for marker and drop the box on it.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None   

    Example call:
    ---
    self.search_marker()
        '''
               
 
        # If drone is not detecting and drone is not landing,this happens when it is either ascendind to detect or descending
        if(self.detected==0 and self.search_change==1): 
            self.search_setpoints=[]
    
            # If threshold box is reached then give coordinates to descend the drone,self.wait is added so that only one coordinate is given
            if(self.reached==1 and self.wait==0): 
               
                alti=self.setpoints[self.search_index][2]-(16.75-0.35)+0.1
                
                self.search_setpoints.append([self.final_x,self.final_y,alti])
                self.wait=self.wait+1
             
                
        # If drone is detecting, then coordinates are given to get drone in the threshold box
        if(self.detected==1 and self.search_change==1): 
            self.search_setpoints=[]
            
            
            self.search_setpoints.append([self.global_x,self.global_y,self.setpoints[self.search_index][2]])
                        

        self.now=time.time() # Getting the current time

        

        elapsed_time=self.now - self.lastTime # Time elapsed 
        
        
        
        if(elapsed_time>=0.001 and len(self.search_setpoints)==1): # Sample time
            self.pid_search(elapsed_time)
            


            # Setting the tolerance for lattitude,longitude and altitude
            # If the current coordinates is within the bounded range then lat/longt/alt is set to 1,else it is set to 0
            
            if(((self.search_setpoints[0][0]-0.000001717)<self.current_coord[0]) and ((self.search_setpoints[0][0]+0.000001717)>self.current_coord[0])):
                lat=1
            else:
                lat=0 
           
            if(((self.search_setpoints[0][1]-0.0000017487)<self.current_coord[1]) and ((self.search_setpoints[0][1]+0.0000017487)>self.current_coord[1])):
                longt=1
            else:
                longt=0  

            if(((self.search_setpoints[0][2]-0.05)<self.current_coord[2]) and ((self.search_setpoints[0][2]+0.05)>self.current_coord[2])):
                alt=1
            else:
                alt=0 
            
                       
            if(lat+longt+alt==3): 
                self.search_change=1
                if(self.reached==1 and self.wait==1): # If drone has descended after detecting
                               
                    self.search=0       # Quit search
                    self.value=False    # Drop the box
                    

                    # Comparing altiude of current altitude+5,and next altitude    
                    if(self.setpoints[self.search_index+1][2]>self.current_coord[2]+5):
                        alti=self.setpoints[self.search_index+1][2]
                    else:
                        alti=self.current_coord[2]+5

                    self.setpoints.insert(self.r,[self.current_coord[0],self.current_coord[1],alti,0,"DROP"])
                    self.setpoints[self.search_index+2][2]=alti
                            
    
                
            else:
                self.search_change=0


def Sort(dst_lst_order):

    '''
    Purpose:
    ---
    Function to sort the list in descending order 

    Input Arguments:
    ---
    'dst_lst_order' : [ list ]
        A list containg distance of drone from various setpoints and also their delivery grid id
        Example: [[3.4,"A2"],[4.5,"C2"]]

    Returns:
    ---
    'dst_lst_order' : [ list ]  
        A list containg distance of drone from various setpoints and their delivery grid id in descending order 

    Example call:
    ---
    Sort([[3.4,"A2"],[4.5,"C2"]])
    '''
               

    dst_lst_order.sort(key=lambda x:x[0])
    dst_lst_order.reverse()
    return dst_lst_order



                
if __name__ == '__main__':  


    '''
    Purpose:
    ---
    Main function,creates object of class Edrone and calls the pid function continously at a fixed frequency.
    Also contains various list for deciding sequence of delivery and creates sequenced_manifest.csv
          
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

    




    # List to append the sequence of delivery in form of grid id,eg,["A2","B1","C3"]    
    delivery_lst=[] 
    # GPS coordinate of delivery grid is stored in this list along with its grid id
    delivery_grid=[[18.9998102845,72.000142461,16.757981,"A1"],[18.9998102845,72.000156706,16.757981,"A2"],[18.9998102845,72.000170951,16.757981,"A3"],[18.999823837,72.000142461,16.757981,"B1"],[18.999823837,72.000156706,16.757981,"B2"],[18.999823837,72.000170951,16.757981,"B3"],[18.999837389,72.000142461,16.757981,"C1"],[18.999837389,72.000156706,16.757981,"C2"],[18.999837389,72.000170951,16.757981,"C3"]] 
    # GPS coordinate of return grid is stored in this list along with its grid id
    return_grid=[[18.9999367615,72.000142461,16.757981,"X1"],[18.9999367615,72.000156706,16.757981,"X2"],[18.9999367615,72.000170951,16.757981,"X3"],[18.999950314,72.000142461,16.757981,"Y1"],[18.999950314,72.000156706,16.757981,"Y2"],[18.999950314,72.000170951,16.757981,"Y3"],[18.999963866,72.000142461,16.757981,"Z1"],[18.999963866,72.000156706,16.757981,"Z2"],[18.999963866,72.000170951,16.757981,"Z3"]] 
    # To store pickup location of delivery parcels from delivery grid in sequence stored in delivery_lst
    delivery_pickup_lst=[]  
    # To store destination of delivery parcel in order set in delivery_lst
    delivery_dst_lst=[]
    # To store coordinate of closest return parcel 
    nearest_parcel_lst=[]
    # To store coordinates of final return destination of return parcel in same order as in nearest_parcel_lst
    return_dst_lst=[]
    # To store data from csv file of deliveries
    csv_list_delivery=[]
    # To store data from csv file of returns 
    csv_list_return=[] 
    # To store distance and grid id of deliveries in descending order according to distance
    dst_lst_order=[]
    # To store distance of all the return parcel from any delivery location 
    dst_lst=[]

    # Lists to store data for sequenced_manifest.csv
    Type_lst=[]
    Origin_lst=[]
    Destination_lst=[]

    with open("/home/utkarsh/catkin_ws/src/vitarana_drone/scripts/bonus.csv","r") as csv_file:
        csv_reader=csv.reader(csv_file)
           
        for line in csv_reader:

            if(line[0].replace(" ","")=="DELIVERY"):
                line[0]=line[0].replace(" ","")
                line[1]=line[1].replace(" ","")
                line[2]=line[2].replace(" ","")
                
                csv_list_delivery.append(line)
            if(line[0].replace(" ","")=="RETURN"):
                line[0]=line[0].replace(" ","")
                line[1]=line[1].replace(" ","")
                line[2]=line[2].replace(" ","")
                
                csv_list_return.append(line)

    
    e_drone = Edrone() # Object of Edrone class



    # Appending distance and delivery grid id in dst_lst_order list
    for index_i in range(0,len(csv_list_delivery)):
        txt_lst=csv_list_delivery[index_i][2].split(";")
                 
        X=float(txt_lst[0])
        Y=float(txt_lst[1])
        Z=float(txt_lst[2])
        # Calculating total latitude distance between current setpoint and destination setpoint in metres
        x=(abs(18.9998887906-X))*110692.0702932625 
        # Calculating total longitude distance between current setpoint and destination setpoint in metres
        y=(abs(72.0002184407-Y))*105292.0089353767 
                        
        path_distance=(x**2+y**2)**0.5
        dst_lst_order.append([path_distance,csv_list_delivery[index_i][1]])


    # Getting the list in descending order
    dst_lst_order=Sort(dst_lst_order)   
    
    
    # Arranging grid id for delivery according to dst_lst_order list
    for index_i in range(0,len(dst_lst_order)):
        delivery_lst.append(dst_lst_order[index_i][1])

   

    for index_i in range(0,len(delivery_lst)):
        for index_j in range(0,len(delivery_grid)):
            if(delivery_lst[index_i]==delivery_grid[index_j][3]):
                delivery_pickup_lst.append([delivery_grid[index_j][0],delivery_grid[index_j][1],delivery_grid[index_j][2],0,delivery_lst[index_i]])      # Appending coordinates to pick parcel from grid
 
        for index_k in range(0,len(delivery_lst)):
            if(delivery_lst[index_i]==csv_list_delivery[index_k][1]):
                 txt_lst=csv_list_delivery[index_k][2].split(";")
                 
                 X=float(txt_lst[0])
                 Y=float(txt_lst[1])
                 Z=float(txt_lst[2])
                 delivery_dst_lst.append([X,Y,Z,0,delivery_lst[index_i]])
                              # Appending coordinates of final location of delivery parcels
   
    for index_i in range(0,len(delivery_dst_lst)):
        dst_lst=[]
        for index_j in range(0,len(csv_list_return)):
            txt_lst=csv_list_return[index_j][1].split(";")
            X=float(txt_lst[0])
            Y=float(txt_lst[1])
            dst_lst.append((((X-delivery_dst_lst[index_i][0])*110692.0702932625)**2+((Y-delivery_dst_lst[index_i][1])*105292.0089353767)**2)**0.5)  
        txt_lst=csv_list_return[np.argmin(dst_lst)][1].split(";") 
        X=float(txt_lst[0])
        Y=float(txt_lst[1])
        Z=float(txt_lst[2])
        nearest_parcel_lst.append([X,Y,Z,0,csv_list_return[np.argmin(dst_lst)][2]])# Appending coordinates of nearest return parcels
        del csv_list_return[np.argmin(dst_lst)] 

    for index_i in range(0,len(nearest_parcel_lst)):
        for index_j in range(0,len(return_grid)):
            if(nearest_parcel_lst[index_i][4]==return_grid[index_j][3]):
               
                return_dst_lst.append([return_grid[index_j][0],return_grid[index_j][1],return_grid[index_j][2],0,return_grid[index_j][3]])  # Appending coordinates of grid coordinates of return parcels
    
    

    for index_i in range(0,len(delivery_lst)):
        e_drone.setpoints.append([delivery_pickup_lst[index_i][0],delivery_pickup_lst[index_i][1],delivery_pickup_lst[index_i][2]+2,delivery_pickup_lst[index_i][3],delivery_pickup_lst[index_i][4]])# Hover the drone 2 metres above the first delivery grid id
        e_drone.setpoints.append([delivery_pickup_lst[index_i][0],delivery_pickup_lst[index_i][1],delivery_pickup_lst[index_i][2]-0.03,delivery_pickup_lst[index_i][3],delivery_pickup_lst[index_i][4]])# Pick that parcel
        e_drone.setpoints.append([delivery_pickup_lst[index_i][0],delivery_pickup_lst[index_i][1],delivery_pickup_lst[index_i][2]+10,delivery_pickup_lst[index_i][3],delivery_pickup_lst[index_i][4]])# Hover 10 metres above thet location after picking the parcel
        e_drone.setpoints.append([delivery_dst_lst[index_i][0],delivery_dst_lst[index_i][1],delivery_dst_lst[index_i][2]+16.75-0.35,2,delivery_dst_lst[index_i][4]])# Move to the destination of that parcel
#############################################################################################################################
        # For sequenced_manifest.csv
        Type_lst.append("Delivery")
        Origin_lst.append(delivery_dst_lst[index_i][4])
        Destination_lst.append(str(delivery_dst_lst[index_i][0])+";"+str(delivery_dst_lst[index_i][1])+";"+str(delivery_dst_lst[index_i][2]+16.75))
##############################################################################################################################     
        e_drone.target_setpoints.append([delivery_dst_lst[index_i][0],delivery_dst_lst[index_i][1],delivery_dst_lst[index_i][2]+16.75-0.35,2,delivery_dst_lst[index_i][4]])# appending the point from where to search the marker
        e_drone.setpoints.append([nearest_parcel_lst[index_i][0],nearest_parcel_lst[index_i][1],nearest_parcel_lst[index_i][2]+10,nearest_parcel_lst[index_i][3],nearest_parcel_lst[index_i][4]]) # To hover 5 metres above closest return parcel
        e_drone.setpoints.append([nearest_parcel_lst[index_i][0],nearest_parcel_lst[index_i][1],nearest_parcel_lst[index_i][2]-0.03,nearest_parcel_lst[index_i][3],nearest_parcel_lst[index_i][4]])# Pick the closest return parcel    
        e_drone.setpoints.append([return_dst_lst[index_i][0],return_dst_lst[index_i][1],return_dst_lst[index_i][2]+10,return_dst_lst[index_i][3],return_dst_lst[index_i][4]])# Hover 10 metres above the parcel dropping grid
        e_drone.setpoints.append([return_dst_lst[index_i][0],return_dst_lst[index_i][1],return_dst_lst[index_i][2],return_dst_lst[index_i][3],return_dst_lst[index_i][4]])  # Getting close to the grid 
        e_drone.setpoints.append([return_dst_lst[index_i][0],return_dst_lst[index_i][1],return_dst_lst[index_i][2],return_dst_lst[index_i][3],"DROP"]) # Drop the parcel
##############################################################################################################################
        # For sequenced_manifest.csv
        Type_lst.append("Return")
        Origin_lst.append(str(nearest_parcel_lst[index_i][0])+";"+str(nearest_parcel_lst[index_i][1])+";"+str(nearest_parcel_lst[index_i][2]))
        Destination_lst.append(nearest_parcel_lst[index_i][4])
        

##############################################################################################################################          
  
    # Making sequenced_manifest.csv file
    df=pd.DataFrame({"a":Type_lst,"b":Origin_lst,"c":Destination_lst})
    df.to_csv("sequenced_manifest.csv",index=False,header=False)

    # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    r = rospy.Rate(e_drone.sample_frequency)  
    rospy.loginfo("started")
    # To pass index for setpoints
    i=0 
   
    while not rospy.is_shutdown():
        # If no obstacle is detected and setpoints_local list is empty
        if((e_drone.back + e_drone.front + e_drone.left + e_drone.right==0) and (len(e_drone.setpoints_local)==0) and e_drone.search==0 and e_drone.end==0):               
            i=e_drone.move_global(i) # Current index number is passed and is returned after it is updated
        # If an obstacle is detected
        if((e_drone.back + e_drone.front + e_drone.left + e_drone.right==1) and e_drone.end==0):
            e_drone.move_local() # To move the drone along the obstacle if only obstacle is detected only in one side
        # If obstacle is not detected but setpoints_local list is not empty
        if(e_drone.back + e_drone.front + e_drone.left + e_drone.right==0) and ((len(e_drone.setpoints_local)!=0) and e_drone.end==0):
            e_drone.local_escape() # To move towards the final goal once obstacle has ended
        # If we have to search for marker  
        if((e_drone.back + e_drone.front + e_drone.left + e_drone.right==0) and (len(e_drone.setpoints_local)==0) and e_drone.search==1 and e_drone.end==0):
            e_drone.search_marker() # To search for marker and move towards its threshold box

        if(e_drone.end==1):
            break # Ending the script

        
         
        r.sleep()             


     


