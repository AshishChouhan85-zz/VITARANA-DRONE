#!/usr/bin/env python




# Importing the required libraries

from vitarana_drone.msg import edrone_cmd
#from vitarana_drone.msg import MarkerData
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
        rospy.init_node('position_controller') # Initializing ros node with name position_control

        # Coordinates of main setpoints stored in a list
        self.setpoints =[]
        self.target_setpoints=[] # Setpoints after which we need to search for marker
        self.setpoints_local=[]  # Setpoints for avoiding obstacle
        self.search_setpoints=[] # Setpoints for searching for the marker
        self.index_list=[]       # Setpoints which keep index of dynamically added setpoints until we reach the destination


        self.change=1            # Flag to give permission to append coordinates in local setpoints
        self.escape_change=1     # Flag to give permission to append coordinates to escape the obstacle

        self.lastTime=0.0        # To store the last time
        self.now=0.0             # To store current time

        self.r=0     # To store value of current index number of global setpoints
        self.i=0     # To manipulate index of target setpoints

        self.search=0                      # Flag to allow searching or quit searching
        self.search_index=0                # To store previous previous index of global list
        self.detected=0                    # Flag which tells whether the marker is detcted or not to give coordinates to move towards the marker
        self.search_change=1               # Flag to give permission to append coordinates in search setpoints
        self.focal_length =238.350718905   # Focal length of camera used
        self.global_x=0                    # To store latitude that will lead to marker
        self.global_y=0                    # To store longitude that will lead to marker
        self.lat=0                         # Flag which becomes 1 when x distance of drone from centre of marker becomes less than 0.15 metres 
        self.longt=0                       # Flag which becomes 1 when y distance of drone from centre of marker becomes less than 0.2 metres 
        self.final_x=0                     # Flag which store latitude for which x distance becomes less than 0.15 metres
        self.final_y=0                     # Flag which store longitude for which y distance becomes less than 0.2 metres
        self.reached=0                     # Flag which gives setpoint to decrease altitude of drone after it reaches the threshold box
        self.wait=0                        # So that only 1 setpoint is given by flag self.reached
        self.aligned=0                     # Flag to tell whether drone is inside threshold box or not
        self.alti=0
        self.only_once=0


        self.path_dist=0    # To store distance between two setpoints
        self.path="null"    # To store whether absolute value of latitude is more or longitude
        self.sign_x=1       # Sign of latitude in case of destination coordinate minus current coordinate
        self.sign_y=1       # Sign of longitude in case of destination coordinate minus current coordinate 

        self.obstacle="null" # To store whether obstacle is in front,back,left or right

        self.value = True    # To store whether gripper will be activated or not

        self.count=0         # So that only one coordinate is given to escape the obstacle

        self.end=0           # To end the script
       

        self.back=0 # To store distance of obstacle from back
        self.front=0 # To store distance of obstacle from front
        self.left=0 # To store distance of obstacle from left
        self.right=0 # To store distance of obstacle from right

        self.obstacle_dist_x=0 # To store distance from obstacle in latitude
        self.obstacle_dist_y=0 # To store distance from obstacle in longitude      

        self.surface_dst=0 
        

        """
                                              BACK
                                                |
                                                |
                                  RIGHT---------|----------LEFT
                                                |
                                                |
                                              FRONT

        """


       


        # List for storing current [latitude,longitude,altitude]
        self.current_coord = [0.0,0.0,0.0]

        # Variables for storing error in latitude , longitude and altitude
        #self.marker_data=MarkerData()
        #self.hold = Float32()
        #self.z_error = Float32()

        #self.marker_data.marker_id=2
        #self.marker_data.err_x_m=0.0
        #self.marker_data.err_y_m=0.0
        #self.z_error.data = 0.0
              
 
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

        #self.near = False # To store whether object can be picked or not

        # Initialising lists to store errors
        
        self.current_error=[0, 0, 0]       # To store current error in [latitude,longitude,altitude] format
        self.previous_error=[0, 0, 0]      # To store previous error of global setpoints in [latitude,longitude,altitude] format
        self.previous_error_local=[0, 0, 0]# To store previous error of local setpoints in [latitude,longitude,altitude] format        

        # Storing max and min values for pwm

        self.max_values = [2000,2000,2000,2000] # Max limit corresponding to [rcRoll,rcPitch,rcYaw,rcThrottle]
        self.min_values = [1000,1000,1000,1000] # Min limit corresponding to [rcRoll,rcPitch,rcYaw,rcThrottle]
        
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
        #self.marker_data_pub=rospy.Publisher('/edrone/marker_data',MarkerData,queue_size=1)

        # SUBSCRIBERS

        rospy.Subscriber('/edrone/gps',NavSatFix,self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top',LaserScan,self.top_callback)
        rospy.Subscriber('/edrone/range_finder_bottom',LaserScan,self.bottom_callback)
        rospy.Subscriber("pixel_coord",String,self.marker_coord)

        # SERVICES

        self.gp_srv = rospy.ServiceProxy('/edrone/activate_gripper',Gripper) # Creating connection to service
        self.gp_srv.wait_for_service() # Waiting for service to be running

       
######################### Function to get current lattitude,longitude and altitude using gps ###################################################

    def gps_callback(self,msg):

        self.current_coord[0] = msg.latitude
        self.current_coord[1] = msg.longitude
        self.current_coord[2] = msg.altitude  

############################# Function to get distance of surface below the drone ###########################################

    def bottom_callback(self,msg):
        if(msg.ranges[0]>=0.5):
            self.surface_dst=msg.ranges[0]

        

############################# Functions to tune pid values ###########################################################

    #def x_set_pid(self, x):
        #self.Kp[0] = x.Kp*10000  
        #self.Ki[0] = x.Ki*1000 
        #self.Kd[0] = x.Kd*10000 


    def y_set_pid(self, y):
        self.Kp[0] = y.Kp*10000  
        self.Ki[0] = y.Ki*100
        self.Kd[0] = y.Kd*10000 
 
    
    #def z_set_pid(self, z):
        #self.Kp_hold[2] = z.Kp  
        #self.Ki_hold[2] = z.Ki
        #self.Kd_hold[2] = z.Kd 

######################### Function to activate gripper and it also returns True if box is attached to gripper #####################################################

    def hold_box(self,value):
        req=GripperRequest()                  
        
        req.activate_gripper=value
               
        resp=self.gp_srv(req)  
               
           
        return resp.result



##################### Function to plan the path for delivery of parcel #################################################################

    def path_planner(self,index_no,x,y): 

        self.index_list=[] # Emptying the list to give new coordinates for each run

        r=500  # This is the distance of between dynamically added setpoints
            
        
        theta=math.atan(x/y) # Angle in radians,which is always positive     
         

        d=self.path_dist # Storing total distance between current setpoint and destination setpoint

        n=int(d/r) # Getting required number of setpoints to reach close to destination

        # Saving sign of destination setpoint minus current setpoint

       

        alti=self.setpoints[index_no-1][2]

        
        # Giving setpoints
     
        for i in range(1,n+1):

            lati=self.setpoints[index_no-1][0]+(r*(math.sin(theta))*self.sign_x)/110692.0702932625
            longi=self.setpoints[index_no-1][1]+(r*(math.cos(theta))*self.sign_y)/105292.0089353767
                
            self.setpoints.insert(index_no,[lati,longi,alti,1])
            self.index_list.append(index_no)
            index_no=index_no+1

        # Storing whether latitude is larger or longitude is larger as it is helpful in obstacle avoidance  
    
       
        
            
  
            

    

########### Function to detect obstacles in front,right,back and left direction and accordingly sets flags for deciding coordinates to avoid obstacles ############

    def top_callback(self,msg):
        self.back=0
        self.front=0
        self.left=0
        self.right=0

        left=msg.ranges[0]
        front=msg.ranges[1]
        right=msg.ranges[2]
        back=msg.ranges[3]

        if(self.r!=len(self.setpoints) and self.setpoints[self.r][3]==1 and self.search==0): # So that obstacle is not detected for certain setpoints and for certain events
            

            if(back<=8 and back>=0.4): # If obstacle is detected in back 
                self.back=1
                        
                self.obstacle_dist_x=back

            if(front<=9 and front>=0.4): # If obstacle is detected in back 
                self.front=1
                        
                self.obstacle_dist_x=front
                
            

            if(left<=8 and left>=0.4): # If obstacle is detected in left
                self.left=1
                       
                self.obstacle_dist_y=left
        
            if(right<=8 and right>=0.4 ): # If obstacle is detected in right
                self.right=1
                       
                self.obstacle_dist_y=right

        if(self.search==1): # To turn sensors off when drone is searching for marker
            self.back=0
            self.front=0
            self.left=0
            self.right=0 

        
      

##################################### Function to get marker coordinates ######################################################################

    def marker_coord(self,msg):
        
       if(msg.data!="NO MARKER"):   # If some pixel coordinate is achieved
            txt=msg.data
            txt_lst=txt.split(",")
            X=txt_lst[0]
            Y=txt_lst[1]
            X=int(X)         # Getting x coordinate of centre pixel of marker
            Y=int(Y)         # Getting y coordinate of centre pixel of marker
            
            Z_m=16.75        # Distance upto which drone will hover from given global setpoint to detect marker
            dist_x=(X*Z_m)/self.focal_length      # Getting x distance from centre of marker to drone
            dist_y=(Y*Z_m)/self.focal_length      # Getting y distance from centre of marker to drone
            
            dist_x=abs(dist_x-14.065193393445954)   # Removing constant error from x distance
            dist_y=abs(dist_y-14.336017175437686) # Removing constant error from y distance
            if(Y>200):                            
                sign_y=1
            else:
                sign_y=-1
            
            if(X>200):
                sign_x=1
            else:
                sign_x=-1                        # Giving proper sign for correct latitude and longitude calculation

            self.global_x=self.current_coord[0]+(dist_x/110692.0702932625)*sign_x
            self.global_y=self.current_coord[1]+(dist_y/105292.0089353767)*sign_y # Getting approx coordinates to centre of marker  
            if(self.surface_dst>=16.65 and self.surface_dst<=16.85 and self.search==1 and self.only_once==0):
                self.detected=1
                self.only_once=1 
                          
           
                       

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

            if(self.lat==1):
                self.global_x=self.final_x

            if(self.longt==1):
                self.global_y=self.final_y     # All these are done to store latitude and longitude for which drone will be inside threshold box
                        

            if((self.lat+self.longt)==2 and dist_x<=0.15 and dist_y<=0.2): # When drone is inside the threshold,then
                self.detected=0    # Stop moving towards marker as we have reached it
                self.reached=1     # Tell drone that it has reached the marker and now decrease its altitude
                self.aligned=1     # Tell drone that it is inside threshold box
                self.error_dist_x=dist_x 
                self.error_dist_y=dist_y
           
      
        
            
       

  
########################################### Function for maintaining pid of global setpoints #########################################################           
    
    def pid_global(self,index_no,elapsed_time):

        # Calculating error in orientation

        self.current_error[0] = self.setpoints[index_no][0] - self.current_coord[0]
        self.current_error[1] = self.setpoints[index_no][1] - self.current_coord[1]

        

        self.current_error[2] = self.setpoints[index_no][2] - self.current_coord[2]

        hold=self.hold_box(self.value) # To give value to activate/deactivate gripper,hold stores True/False if box is attached to gripper or not
        #print(hold)     
            
                      
               
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


  
##################################### Function for maintaining pid values of local setpoints #############################################################  
    
    def pid_local(self,elapsed_time):

        # Calculating error in orientation
                
        self.current_error[0] = self.setpoints_local[0][0] - self.current_coord[0]
        self.current_error[1] = self.setpoints_local[0][1] - self.current_coord[1]
        self.current_error[2] = self.setpoints_local[0][2] - self.current_coord[2]

        hold=self.hold_box(self.value) # To give value to activate/deactivate gripper,hold stores True/False if box is attached to gripper or not
              
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


###################################### Function for maintaing pid values while drone is searching for marker ################

    def pid_search(self,elapsed_time):

        # Calculating error in orientation
                
        self.current_error[0] = self.search_setpoints[0][0] - self.current_coord[0]
        self.current_error[1] = self.search_setpoints[0][1] - self.current_coord[1]
        self.current_error[2] = self.search_setpoints[0][2] - self.current_coord[2]

        #hold=self.hold_box(self.value) # To give value to activate/deactivate gripper,hold stores True/False if box is attached to gripper or not
        hold=False        
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
    


###################################### Function to traverse the required setpoints of global list ############################################################

    def move_global(self,index_no):

        index_no=self.r # Passing the value of current index number so that there is no scope of error as sometimes None also gets passed

        self.now=time.time() # Getting the current time

        

        elapsed_time=self.now - self.lastTime # Time elapsed 
        
        
        
        if(elapsed_time>=0.001): # Sample time
            

            if(index_no!=len(self.setpoints) and index_no<len(self.setpoints)):
                self.pid_global(index_no,elapsed_time)
                

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


                # If first setpoint is reached then increment the index so that next setpoint is selected,else it is not incremented and remains same                
               
                    
                # This is used after the first setpoint is reached
                if(lat+longt+alt==3):
                    #self.value=True
                    if(self.setpoints[index_no][4]=="DROP"):
                        self.value=False
                    else:
                        self.value=True

                    
                    if(self.target_setpoints[self.i]==self.setpoints[index_no]): # If drone reachs a dynamically inserted setpoint then do not search for marker                  
                        self.search=1
                        self.only_once=0
                        self.search_index=index_no
                        self.lat=0
                        self.longt=0
                        self.wait=0
                        self.detected=0
                        self.reached=0
                        self.aligned=0  # Initial value of flags given before search of marker is done
                        self.i=self.i+1
                        if(self.i>=len(self.target_setpoints)):
                            self.i=len(self.target_setpoints)-1
                    else:
                        self.search=0
                       
                       
                    index_no=index_no+1

                    
                   

                    #if(index_no<=(len(self.setpoints)-1) and self.search!=1):

                        #x=(abs(self.setpoints[index_no][0]-self.setpoints[index_no-1][0]))*110692.0702932625 # Calculating total latitude distance between current setpoint and destination setpoint in metres
                        #y=(abs(self.setpoints[index_no][1]-self.setpoints[index_no-1][1]))*105292.0089353767 # Calculating total longitude distance between current setpoint and destination setpoint in metres
                        
                        #self.path_dist=(x**2+y**2)**0.5 # Calculating distance between current setpoint and destination setpoint in metres  

                        #if(x>y):
                            #self.path="X"
                        #else:
                            #self.path="Y"

                        #if((self.setpoints[index_no][0]-self.setpoints[index_no-1][0])>0):
                            #self.sign_x=1
                        #else:
                            #self.sign_x=-1         

                        #if((self.setpoints[index_no][1]-self.setpoints[index_no-1][1])>0):
                            #self.sign_y=1
                        #else:
                            #self.sign_y=-1

                        
                
                        
                        #if(self.path_dist>500): # If this distance is more than 30 metres then make a path to follow
                            #self.path_planner(index_no,x,y)          
                    
                    
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

############################## Function to move the drone along the obstacle if obstacle is detected only in one direction ##########################################
                           
    def move_local(self):

        self.count=1
        
        if(self.change==1): # Value of change is 1 when it is beginning,required local setpoint is reached or when during escape drone encounters an obstacle
            
            self.setpoints_local=[] # List is cleared so that index number remains 0 
 
            if(self.path=="X"):           
                if(self.back==1 and self.sign_x!=1): 
                    lati=self.current_coord[0] + ((5-self.obstacle_dist_x)/110692.0702932625) # For maintaining a fixed distance from obstacle
                    longi=self.current_coord[1]+(5.5/105292.0089353767) # For moving along obstacle
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
                    lati=self.current_coord[0] + ((5-self.obstacle_dist_x)/110692.0702932625) # For maintaining a fixed distance from obstacle
                    longi=self.current_coord[1]+(5.5/105292.0089353767)*self.sign_y # For moving along obstacle
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

################################# Function to escape the obstacle when it has ended and to move towards the final goal ############################################
            
    def local_escape(self): # This function helps to reach the local setpoint even when obstacle has gone but local setpoint is not reached
           
        

        

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
   
                    # Skipping the dynamically added setpoint if we have crossed it already due to obstacle avoidance                     
                    if(len(self.setpoints_local)!=0):
                        for i in range(0,len(self.index_list)):
                            if(self.r==self.index_list[i]):

                                if(self.sign_x>0):
                                    if(lati>=self.setpoints[self.r][0]):
                                        self.r=self.r+1
                                       
                                if(self.sign_x<0):
                                    if(lati<=self.setpoints[self.r][0]):
                                        self.r=self.r+1
                                        
                                
                            


                
                

################################# Function to give coordinates to search for marker ##################################################################
           
    def search_marker(self):
 

        if(self.detected==0 and self.search_change==1): # If drone is not detecting and drone is not landing,this happens when it is either ascendind to detect or descending
            self.search_setpoints=[]
    
            if(self.reached==0):   # If drone has not reached the threshold box,that means it is increasing its height
                self.alti=self.current_coord[2]+(16.75-self.surface_dst)
                #print(self.alti)
                self.search_setpoints.append([self.setpoints[self.search_index][0],self.setpoints[self.search_index][1],self.alti])
                


            if(self.reached==1 and self.wait==0):  # If threshold box is reached then give coordinates to descend the drone,self.wait is added so that only one coordinate is given
               
                self.alti=self.current_coord[2]-self.surface_dst+0.5
                
                self.search_setpoints.append([self.final_x,self.final_y,self.alti])
                self.wait=self.wait+1
             
                

        if(self.detected==1 and self.search_change==1): # If drone is detecting, then coordinates are given to get drone in the threshold box
            self.search_setpoints=[]
            
            
            self.search_setpoints.append([self.global_x,self.global_y,self.alti])
                        

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
            
                       
            if(lat+longt+alt==3): # If drone is landing then program does not go inside it as no coordinate is given then,only throttle values are given
                self.search_change=1
                if(self.reached==1 and self.wait==1): # If drone has descended after detecting
                               
                    self.search=0       # Quit search
                    self.value=False    # Drop the box
                    

                        
                    if(self.setpoints[self.search_index+1][2]>self.current_coord[2]+5):
                        alti=self.setpoints[self.search_index+1][2]
                    else:
                        alti=self.current_coord[2]+5
                    self.setpoints.insert(self.r,[self.current_coord[0],self.current_coord[1],alti,0,"NULL"])
                    self.setpoints[self.search_index+2][2]=alti
                            
    
                
            else:
                self.search_change=0

#################################### Sorting function ###########################################################

def Sort(dst_lst_order):
    dst_lst_order.sort(key=lambda x:x[0])
    dst_lst_order.reverse()
    return dst_lst_order



       
#################################################### Main function #################################################################################
                
if __name__ == '__main__':
    
    delivery_lst=[] # List to decide the sequence of delivery
    delivery_grid=[[18.9998102845,72.000142461,16.757981,"A1"],[18.9998102845,72.000156706,16.757981,"A2"],[18.9998102845,72.000170951,16.757981,"A3"],[18.999823837,72.000142461,16.757981,"B1"],[18.999823837,72.000156706,16.757981,"B2"],[18.999823837,72.000170951,16.757981,"B3"],[18.999837389,72.000142461,16.757981,"C1"],[18.999837389,72.000156706,16.757981,"C2"],[18.999837389,72.000170951,16.757981,"C3"]]  # Coordinates of all the delivery parcels
    return_grid=[[18.9999367615,72.000142461,16.757981,"X1"],[18.9999367615,72.000156706,16.757981,"X2"],[18.9999367615,72.000170951,16.757981,"X3"],[18.999950314,72.000142461,16.757981,"Y1"],[18.999950314,72.000156706,16.757981,"Y2"],[18.999950314,72.000170951,16.757981,"Y3"],[18.999963866,72.000142461,16.757981,"Z1"],[18.999963866,72.000156706,16.757981,"Z2"],[18.999963866,72.000170951,16.757981,"Z3"]] # Coordinates of all the return parcels
    delivery_pickup_lst=[]
    delivery_dst_lst=[] # To store destination of delivery parcel in order set in delivery_lst
    nearest_parcel_lst=[] # To store coordinate of closest return parcel 
    return_dst_lst=[] # To store coordinates of final return destination of return parcel in same order as in nearest_parcel_lst
    csv_list_delivery=[] # To store data from csv file
    csv_list_return=[] # To store data from csv file
    dst_lst_order=[]
    dst_lst=[] # To store distance of all the return parcel from any final delivery location
    Type_lst=[]
    Origin_lst=[]
    Destination_lst=[]
    with open("/home/ashish/catkin_ws/src/vitarana_drone/scripts/manifest.csv","r") as csv_file:
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

    for index_i in range(0,len(csv_list_delivery)):
        txt_lst=csv_list_delivery[index_i][2].split(";")
                 
        X=float(txt_lst[0])
        Y=float(txt_lst[1])
        Z=float(txt_lst[2])
        x=(abs(18.9998887906-X))*110692.0702932625 # Calculating total latitude distance between current setpoint and destination setpoint in metres
        y=(abs(72.0002184407-Y))*105292.0089353767 # Calculating total longitude distance between current setpoint and destination setpoint in metres
                        
        path_distance=(x**2+y**2)**0.5
        dst_lst_order.append([path_distance,csv_list_delivery[index_i][1]])
    
    dst_lst_order=Sort(dst_lst_order)    
    
    for index_i in range(0,len(dst_lst_order)):
        delivery_lst.append(dst_lst_order[index_i][1])
        
    #print(delivery_lst)

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
                #return_dst_lst.append([return_grid[index_j][0],return_grid[index_j][1],return_grid[index_j][2]+3,0,return_grid[index_j][3]]) 
                return_dst_lst.append([return_grid[index_j][0],return_grid[index_j][1],return_grid[index_j][2],0,return_grid[index_j][3]])  # Appending coordinates of grid coordinates of return parcels
    
    for index_i in range(0,len(delivery_lst)):
        e_drone.setpoints.append([delivery_pickup_lst[index_i][0],delivery_pickup_lst[index_i][1],delivery_pickup_lst[index_i][2]+2,delivery_pickup_lst[index_i][3],delivery_pickup_lst[index_i][4]])
        e_drone.setpoints.append([delivery_pickup_lst[index_i][0],delivery_pickup_lst[index_i][1],delivery_pickup_lst[index_i][2]-0.03,delivery_pickup_lst[index_i][3],delivery_pickup_lst[index_i][4]])
        e_drone.setpoints.append([delivery_pickup_lst[index_i][0],delivery_pickup_lst[index_i][1],delivery_pickup_lst[index_i][2]+10,delivery_pickup_lst[index_i][3],delivery_pickup_lst[index_i][4]])
        e_drone.setpoints.append([delivery_dst_lst[index_i][0],delivery_dst_lst[index_i][1],delivery_dst_lst[index_i][2]+16.75,2,delivery_dst_lst[index_i][4]]) 
#############################################################################################################################
        Type_lst.append("Delivery")
        Origin_lst.append(delivery_dst_lst[index_i][4])
        Destination_lst.append(str(delivery_dst_lst[index_i][0])+";"+str(delivery_dst_lst[index_i][1])+";"+str(delivery_dst_lst[index_i][2]+16.75))
##############################################################################################################################     
        e_drone.target_setpoints.append([delivery_dst_lst[index_i][0],delivery_dst_lst[index_i][1],delivery_dst_lst[index_i][2]+16.75,2,delivery_dst_lst[index_i][4]])
        e_drone.setpoints.append([nearest_parcel_lst[index_i][0],nearest_parcel_lst[index_i][1],nearest_parcel_lst[index_i][2]+5,nearest_parcel_lst[index_i][3],nearest_parcel_lst[index_i][4]])
        e_drone.setpoints.append([nearest_parcel_lst[index_i][0],nearest_parcel_lst[index_i][1],nearest_parcel_lst[index_i][2]-0.03,nearest_parcel_lst[index_i][3],nearest_parcel_lst[index_i][4]])    
        e_drone.setpoints.append([return_dst_lst[index_i][0],return_dst_lst[index_i][1],return_dst_lst[index_i][2]+10,return_dst_lst[index_i][3],return_dst_lst[index_i][4]])
        e_drone.setpoints.append([return_dst_lst[index_i][0],return_dst_lst[index_i][1],return_dst_lst[index_i][2],return_dst_lst[index_i][3],return_dst_lst[index_i][4]])   
        e_drone.setpoints.append([return_dst_lst[index_i][0],return_dst_lst[index_i][1],return_dst_lst[index_i][2],return_dst_lst[index_i][3],"DROP"]) 
##############################################################################################################################
        Type_lst.append("Return")
        Origin_lst.append(str(nearest_parcel_lst[index_i][0])+";"+str(nearest_parcel_lst[index_i][1])+";"+str(nearest_parcel_lst[index_i][2]))
        Destination_lst.append(nearest_parcel_lst[index_i][4])
        

##############################################################################################################################          
  
    df=pd.DataFrame({"a":Type_lst,"b":Origin_lst,"c":Destination_lst})
    df.to_csv("sequenced_manifest.csv",index=False,header=False)
    
    r = rospy.Rate(e_drone.sample_frequency)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    rospy.loginfo("started")
    i=0 # To pass index for setpoints

    #rospy.Timer(rospy.Duration(1),e_drone.publisher_1hz) # Timer to call this function after 1 second everytime

    while not rospy.is_shutdown():
        if((e_drone.back + e_drone.front + e_drone.left + e_drone.right==0) and (len(e_drone.setpoints_local)==0) and e_drone.search==0 and e_drone.end==0):               
            i=e_drone.move_global(i) # Current index number is passed and is returned after it is updated
        if((e_drone.back + e_drone.front + e_drone.left + e_drone.right==1) and e_drone.end==0):
            e_drone.move_local() # To move the drone along the obstacle if only obstacle is detected only in one side

        if(e_drone.back + e_drone.front + e_drone.left + e_drone.right==0) and ((len(e_drone.setpoints_local)!=0) and e_drone.end==0):
            e_drone.local_escape() # To move towards the final goal once obstacle has ended
            
        if((e_drone.back + e_drone.front + e_drone.left + e_drone.right==0) and (len(e_drone.setpoints_local)==0) and e_drone.search==1 and e_drone.end==0):
            e_drone.search_marker() # To search for marker and move towards its threshold box

        if(e_drone.end==1):
            break

        
         
        r.sleep()             


     


