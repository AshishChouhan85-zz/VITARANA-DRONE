#!/usr/bin/env python




# Importing the required libraries

from vitarana_drone.msg import edrone_cmd
from vitarana_drone.msg import MarkerData
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import math
import rospy
import time
from sensor_msgs.msg import LaserScan


class Edrone():
    
    def __init__(self):
        rospy.init_node('position_controller') # Initializing ros node with name position_control

        # Coordinates of main setpoints stored in a list
        self.setpoints =[[18.9990965925,71.9999050292,23.2],[18.9990965928,72.0000664814,11.75],[18.9993675932,72.0000569892,11.7]]
        self.setpoints_local=[] # Setpoints for avoiding obstacle
        self.search_setpoints=[] # Setpoints for searching for the marker
        self.change=1           # Flag to give permission to append coordinates in local setpoints
       
        self.lastTime=0.0 # To store the last time
        self.now=0.0      # To store current time

        self.r=0 # To store value of current index number

        self.insert=0    # Flag to differentiate between manually added setpoint and dynamically added setpoint

        self.search=0                      # Flag to allow searching or quit searching
        self.search_index=0                # To store previous previous index of global list
        self.detected=0                    # Flag which tells whether the marker is detcted or not to give coordinates to move towards the marker
        self.height=0                      # To give altitude to drone to increase field of view
        self.search_change=1               # Flag to give permission to append coordinates in search setpoints
        self.focal_length =238.350718905   # Focal length of camera used
        self.global_x=0                    # To store latitude that will lead to marker
        self.global_y=0                    # To store longitude that will lead to marker
        self.lat=0                         # Flag which becomes 1 when x distance of drone from centre of marker becomes less than 0.2 metres 
        self.longt=0                       # Flag which becomes 1 when y distance of drone from centre of marker becomes less than 0.2 metres 
        self.final_x=0                     # Flag which store latitude for which x distance becomes less than 0.2 metres
        self.final_y=0                     # Flag which store longitude for which y distance becomes less than 0.2 metres
        self.reached=0                     # Flag which gives setpoint to decrease altitude of drone after it reaches the threshold box
        self.wait=0                        # So that only 1 setpoint is given by flag self.reached
        self.land=0                        # Flag to land the drone once last marker is reached
        self.detector_off=1                # Flag to publish x and y distance error from marker centre when it is necessar
        self.aligned=0                     # Flag to tell whether drone is inside threshold box or not
        self.end=0                         # Flag used so that slow landing is done for a while only

       

        self.back=0 # To store distance of obstacle from back
        self.front=0 # To store distance of obstacle from front
        self.left=0 # To store distance of obstacle from left
        self.right=0 # To store distance of obstacle from right

        self.obstacle_dist_x=0 # To store distance from obstacle in latitude
        self.obstacle_dist_y=0 # To store distance from obstacle in longitude       
        

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
        self.marker_data=MarkerData()
        #self.hold = Float32()
        #self.z_error = Float32()

        self.marker_data.marker_id=2
        self.marker_data.err_x_m=0.0
        self.marker_data.err_y_m=0.0
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
        self.Kd_hold = [13700000, 13700000, 800] 

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

        self.value = False # To store whether gripper will be activated or not

        # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation   step time is 50 ms
        self.sample_frequency = 60  # in hertz

        # PUBLISHERS

        self.rc_pub = rospy.Publisher('/drone_command',edrone_cmd,queue_size=1)
        self.marker_data_pub=rospy.Publisher('/edrone/marker_data',MarkerData,queue_size=1)

        # SUBSCRIBERS

        rospy.Subscriber('/edrone/gps',NavSatFix,self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top',LaserScan,self.top_callback)
        rospy.Subscriber("pixel_coord",String,self.marker_coord)

       
######################### Function to get current lattitude,longitude and altitude using gps ###################################################

    def gps_callback(self,msg):

        self.current_coord[0] = msg.latitude
        self.current_coord[1] = msg.longitude
        self.current_coord[2] = msg.altitude

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

        if(self.r!=len(self.setpoints)): # So that obstacle is not detected while landing of drone

            if(back<=8 and back>=0.4 and (abs(self.setpoints[self.r][0]-self.current_coord[0])*110692.0702932625)>1.5 and (abs(self.setpoints[self.r][1]-self.current_coord[1])*105292.0089353767)>1.5 ): # If obstacle is detected in back and current setpoint is outside the threshold box
                self.back=1
                        
                self.obstacle_dist_x=back

        #if(front<=8 and front>=0.4 and (abs(self.setpoints[self.r][0]-self.current_coord[0])*110692.0702932625)>1.5 and (abs(self.setpoints[self.r][1]-self.current_coord[1])*105292.0089353767)>1.5):
            #self.front=1
                        
            #self.obstacle_dist_x=front
            

            if(left<=8 and left>=0.4 and (abs(self.setpoints[self.r][0]-self.current_coord[0])*110692.0702932625)>1.5 and (abs(self.setpoints[self.r][1]-self.current_coord[1])*105292.0089353767)>1.5): # If obstacle is detected in left
                self.left=1
                       
                self.obstacle_dist_y=left
        
        #if(right<=8 and right>=0.4 and (abs(self.setpoints[self.r][0]-self.current_coord[0])*110692.0702932625)>1.5 and (abs(self.setpoints[self.r][1]-self.current_coord[1])*105292.0089353767)>1.5): # If obstacle is detected in left
            #self.right=1
                       
            #self.obstacle_dist_y=right

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
            dist_x=abs(dist_x-14.1549187994487)   # Removing constant error from x distance
            dist_y=abs(dist_y-14.325193393445954) # Removing constant error from y distance
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
            
           
            
            if(self.current_coord[2]>=self.setpoints[self.search_index][2]+15.98 and self.search==1):
                self.detected=1                 # If required height is reached then turn this on for appending coordinates to reach to marker
                self.detector_off=0             # Turn it off so that distance from marker is published  
                
            else:
                self.detected=0                 # If required height is not reached then do not append

            if((dist_x<=0.1) and (self.detected==1)): 
                self.final_x=self.current_coord[0]
                self.lat=1
            else:
                self.lat=0

            if((dist_y<=0.1) and (self.detected==1)):
                self.final_y=self.current_coord[1]
                self.longt=1
            else:
                self.longt=0

            if(self.lat==1):
                self.global_x=self.final_x

            if(self.longt==1):
                self.global_y=self.final_y     # All these are done to store latitude and longitude for which drone will be inside threshold box
                        

            if((self.lat+self.longt)==2 and dist_x<=0.2 and dist_y<=0.2): # When drone is inside the threshold,then
                self.detected=0    # Stop moving towards marker as we have reached it
                self.reached=1     # Tell drone that it has reached the marker and now decrease its altitude
                self.aligned=1     # Tell drone that it is inside threshold box
                self.error_dist_x=dist_x 
                self.error_dist_y=dist_y
                
            sign_y=-sign_y
            sign_x=-sign_x         # Modifying signs so that they match with global coordinate system

            if(self.detected==1 and self.aligned==0 ): # If drone is still detecting marker and it is not inside threshold box
                self.marker_data.err_x_m=dist_x*sign_x
                self.marker_data.err_y_m=dist_y*sign_y

            if(self.aligned==1):  # When drone is inside threshold box
                self.marker_data.err_x_m=self.error_dist_x*sign_x
                self.marker_data.err_y_m=self.error_dist_y*sign_y

       else:  # If we do not get any pixel coordinate then do not publish the distances
           self.detector_off=1
            
        
            
       

  
########################################### Function for maintaining pid of global setpoints #########################################################           
    
    def pid_global(self,index_no,elapsed_time):

        # Calculating error in orientation

        self.current_error[0] = self.setpoints[index_no][0] - self.current_coord[0]
        self.current_error[1] = self.setpoints[index_no][1] - self.current_coord[1]

        

        self.current_error[2] = self.setpoints[index_no][2] - self.current_coord[2]
            
                      
        hold=False        
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

                # If first setpoint is reached then increment the index so that next setpoint is selected,else it is not incremented and remains same                
               
                    
                # This is used after the first setpoint is reached
                if(lat+longt+alt==3):
                    
                    if(self.insert==1): # If drone reachs a dynamically inserted setpoint then do not search for marker
                        self.insert=0  # Now more setpoints can be added
                        self.search=0
                    else:
                        self.search=1
                        self.search_index=index_no
                        self.lat=0
                        self.longt=0
                        self.wait=0
                        self.detected=0
                        self.reached=0
                        self.height=0
                        self.aligned=0  # Initial value of flags given before search of marker is done
                       
                    index_no=index_no+1
                    
                else:
                    index_no=index_no
                
               

            self.r=index_no # Storing the current index number

            

            return index_no

############################## Function to move the drone along the obstacle if obstacle is detected only in one direction ##########################################
                           
    def move_local(self):
        
        if(self.change==1): # Value of change is 1 when it is beginning,required local setpoint is reached or when during escape drone encounters an obstacle
            
            self.setpoints_local=[] # List is cleared so that index number remains 0 
            if(self.back==1): 
                lati=self.current_coord[0] + ((5-self.obstacle_dist_x)/110692.0702932625) # For maintaining a fixed distance from obstacle
                longi=self.current_coord[1]+(4/105292.0089353767) # For moving along obstacle

            if(self.front==1): 
                lati=self.current_coord[0] - ((7-self.obstacle_dist_x)/110692.0702932625)
                longi=self.current_coord[1]-(7/105292.0089353767)            
            

            if(self.left==1):
                lati=self.current_coord[0]-(4/110692.0702932625)
                longi=self.current_coord[1] + ((5-self.obstacle_dist_y)/105292.0089353767)

            if(self.right==1):
                lati=self.current_coord[0]+(4/110692.0702932625)
                longi=self.current_coord[1] - ((5-self.obstacle_dist_y)/105292.0089353767)

            if(self.setpoints[self.r][2]>=self.current_coord[2]): # If obstacle is detected while moving to a higher altitude than current altitude then a new setpoint is appended before this setpoint with 4 metres above this higher altitude                      
                alti=self.setpoints[self.r][2]+4 
                self.setpoints.insert(self.r,[self.setpoints[self.r][0],self.setpoints[self.r][1],alti])
                self.insert=1 # To tell that a setpoint is added to global setpoint list
            else:
                alti=self.current_coord[2]   # If current goal altitude is less than current altitude and obstacle is detected then remain that altitude

            self.setpoints_local.append([lati,longi,alti])
            
        
       
        self.now=time.time() # Getting the current time

        

        elapsed_time=self.now - self.lastTime # Time elapsed 
        
        
        
        if(elapsed_time>=0.001 and len(self.setpoints_local)==1): # Sample time
            self.pid_local(elapsed_time)


            # Setting the tolerance for lattitude,longitude and altitude
            # If the current coordinates is within the bounded range then lat/longt/alt is set to 1,else it is set to 0
            
            if(((self.setpoints_local[0][0]-0.000001717)<self.current_coord[0]) and ((self.setpoints_local[0][0]+0.000001717)>self.current_coord[0])):
                lat=1
            else:
                lat=0 
           
            if(((self.setpoints_local[0][1]-0.0000017487)<self.current_coord[1]) and ((self.setpoints_local[0][1]+0.0000017487)>self.current_coord[1])):
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
            
            if(((self.setpoints_local[0][0]-0.000001717)<self.current_coord[0]) and ((self.setpoints_local[0][0]+0.000001717)>self.current_coord[0])):
                lat=1
            else:
                lat=0 
           
            if(((self.setpoints_local[0][1]-0.0000017487)<self.current_coord[1]) and ((self.setpoints_local[0][1]+0.0000017487)>self.current_coord[1])):
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

################################# Function to give coordinates to search for marker ##################################################################
           
    def search_marker(self):
 

        if(self.detected==0 and self.search_change==1 and self.land==0): # If drone is not detecting and drone is not landing,this happens when it is either ascendind to detect or descending
            self.search_setpoints=[]

            if(self.reached==0):   # If drone has not reached the threshold box,that means it is increasing its height
                alti=self.setpoints[self.search_index][2]+self.height
                self.search_setpoints.append([self.setpoints[self.search_index][0],self.setpoints[self.search_index][1],alti])
                self.height=self.height+16
                if(self.height>=16):  # Height should not be more than 16 metres
                    self.height=16

            if(self.reached==1 and self.wait==0):  # If threshold box is reached then give coordinates to descend the drone,self.wait is added so that only one coordinate is given

                if(self.r>=len(self.setpoints)):
                    alti=self.setpoints[self.search_index][2]+self.height-6
                else:
                    alti=self.setpoints[self.search_index][2]+self.height-2
                self.search_setpoints.append([self.final_x,self.final_y,alti])
                self.wait=self.wait+1
             
                

        if(self.detected==1 and self.search_change==1 and self.land==0): # If drone is detecting, then coordinates are given to get drone in the threshold box
            self.search_setpoints=[]
            
            
            self.search_setpoints.append([self.global_x,self.global_y,self.setpoints[self.search_index][2]+16])
                        

        self.now=time.time() # Getting the current time

        

        elapsed_time=self.now - self.lastTime # Time elapsed 
        
        
        
        if(elapsed_time>=0.001 and len(self.search_setpoints)==1 and self.land==0): # Sample time
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
            
                       
            if(lat+longt+alt==3 and self.land==0): # If drone is landing then program does not go inside it as no coordinate is given then,only throttle values are given
                self.search_change=1
                if(self.reached==1 and self.wait==1): # If drone has descended after detecting
                    
                    self.detector_off=1 # Then publish nan values
                    self.search=0       # Quit search
                    if(self.r!=len(self.setpoints)):
                        if((self.current_coord[2]>self.setpoints[self.r][2])): # If next setpoint is at lower altitude than drone's current altitude and we have not reached all setpoints then move to next setpoint with current altitude
                            alti=self.current_coord[2]
                            self.setpoints.insert(self.r,[self.setpoints[self.r][0],self.setpoints[self.r][1],alti])
                            self.insert=1
                                

                       

                    if(self.r==len(self.setpoints)): # If all global setpoints have been reached and drone has descended after detecting the marker,then
                        self.land=1 # Land the drone
                        self.search=1 # Stay inside this function
                            
                        
                
            else:
                self.search_change=0

        if(self.land==1):    # When drone is ready to land
            if(self.end==0):
                self.drone_orientation.rcRoll = 1500
                self.drone_orientation.rcPitch = 1500
                self.drone_orientation.rcYaw = 1500
                self.drone_orientation.rcThrottle = 1470
                self.rc_pub.publish(self.drone_orientation)    
                
                time.sleep(4)                               # Descend slowly for some time     

            
            self.drone_orientation.rcThrottle = 1000
            self.rc_pub.publish(self.drone_orientation)      
            self.end=1                                       # Then turn throttle to 0 for all time



#################################### Function to publish marker data at 1 Hz #########################################            
            
    def publisher_1hz(self,event=None):

        if(self.detector_off==1):                  # If detector is off then NaN value is published
            self.marker_data.err_x_m=float("NaN")
            self.marker_data.err_y_m=float("NaN")
            

        if(self.search==1):                      # When drone is searching for marker
            if(self.reached==0):                 # When it is ascending
                if(self.setpoints[self.search_index][1]==71.9999050292):
                    self.marker_data.marker_id=2
                if(self.setpoints[self.search_index][1]==72.0000664814):
                    self.marker_data.marker_id=1
                if(self.setpoints[self.search_index][1]==72.0000569892):
                    self.marker_data.marker_id=3
            if(self.reached==1 and self.r!=len(self.setpoints)): # When it is descending
                if(self.setpoints[self.r][1]==71.9999050292):
                    self.marker_data.marker_id=2 
                if(self.setpoints[self.r][1]==72.0000664814): 
                    self.marker_data.marker_id=1
                if(self.setpoints[self.r][1]==72.0000569892):
                    self.marker_data.marker_id=3

        if(self.search==0):                     # When drone is not searching for marker and moving towards a global setpoint
            if(self.setpoints[self.r][1]==71.9999050292):
                self.marker_data.marker_id=2 
            if(self.setpoints[self.r][1]==72.0000664814): 
                self.marker_data.marker_id=1
            if(self.setpoints[self.r][1]==72.0000569892):
                self.marker_data.marker_id=3
                         
        self.marker_data_pub.publish(self.marker_data)          

############################################ Main function #########################################################
                
if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_frequency)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    rospy.loginfo("started")
    i=0 # To pass index for setpoints

    rospy.Timer(rospy.Duration(1),e_drone.publisher_1hz) # Timer to call this function after 1 second everytime

    while not rospy.is_shutdown():
        if((e_drone.back + e_drone.front + e_drone.left + e_drone.right==0) and (len(e_drone.setpoints_local)==0) and e_drone.search==0):               
            i=e_drone.move_global(i) # Current index number is passed and is returned after it is updated
        if((e_drone.back + e_drone.front + e_drone.left + e_drone.right==1) ):
            e_drone.move_local() # To move the drone along the obstacle if only obstacle is detected only in one side

        if(e_drone.back + e_drone.front + e_drone.left + e_drone.right==0) and ((len(e_drone.setpoints_local)!=0)):
            e_drone.local_escape() # To move towards the final goal once obstacle has ended
            
        if((e_drone.back + e_drone.front + e_drone.left + e_drone.right==0) and (len(e_drone.setpoints_local)==0) and e_drone.search==1):
            e_drone.search_marker() # To search for marker and move towards its threshold box

        
            
        r.sleep()             


     


