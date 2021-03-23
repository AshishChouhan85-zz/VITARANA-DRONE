#!/usr/bin/env python




# Importing the required libraries

from vitarana_drone.msg import edrone_cmd
from std_msgs.msg import Float32
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from vitarana_drone.srv import *
import math
import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class Edrone():
    
    def __init__(self):
        rospy.init_node('position_controller') # initializing ros node with name position_control

        

        # Coordinates of setpoints stored in a list
        self.setpoints =[[19.0009248718, 71.9998318945 ,26.0],[19.0007046575, 71.9998955286 ,26.0],[19.0007046575, 71.9998955286 ,22.1199967919],[19.0007046575, 71.9998955286 ,26.0]] #global setpoints
        self.setpoints_local=[]
        self.change=1
       
        self.lastTime=0.0 # To store the last time

        self.r=0 # To store value of current index number
        self.c=1 # To store number of times we will take destination coordinate 

        self.lat_move=1
        self.lat_stay=1
        self.long_move=1
        self.long_stay=1

        self.obstacle_dist_x=0
        self.obstacle_dist_y=0        
        
        self.x1=1
        self.x2=1
        self.esc_x=0

        self.y1=1
        self.y2=1
        self.esc_y=0

        # List for storing current [latitude,longitude,altitude]
        self.current_coord = [0.0,0.0,0.0]

        # Variables for storing error in latitude , longitude and altitude
        #self.x_error = Float32()
        #self.y_error = Float32()
        #self.hold = Float32()
        #self.z_error = Float32()

        #self.x_error.data = 0.0
        #self.y_error.data = 0.0
        #self.z_error.data = 0.0
              
 
        # Declaring drone_orientation of message type edrone_cmd and initialising values

        self.drone_orientation = edrone_cmd()

        # Creating object for service
 
        

        # initial setting of Kp, Kd and ki for [latitude, longitude, altitude]

        self.Kp = [1750000, 1730000,  110] #120.1
        self.Ki = [0.0, 0.0, 5.0]
        self.Kd = [13700000, 13700000, 500] #303.5

        # initial setting of Kp, Kd and ki for [latitude, longitude, altitude] while box is held by drone

        self.Kp_hold = [1750000, 1730000,  300] #165 #
        self.Ki_hold = [0.0, 0.0, 5.0]   #6
        self.Kd_hold = [13700000, 13700000, 800] #380 #

        #self.near = False # To store whether object can be picked or not

        # Initialising lists to store errors
        
        self.current_error=[0, 0, 0]       # To store current error in [latitude,longitude,altitude] format
        self.previous_error=[0, 0, 0]      # To store previous error in [latitude,longitude,altitude] format
        self.previous_error_local=[0, 0, 0]        

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
        #self.x_pub = rospy.Publisher('/x_error', Float32, queue_size=1)
        #self.y_pub = rospy.Publisher('/y_error', Float32, queue_size=1)
        #self.z_pub = rospy.Publisher('/z_error', Float32, queue_size=1)
        

        # SUBSCRIBERS

        rospy.Subscriber('/edrone/gps',NavSatFix,self.gps_callback)
        #rospy.Subscriber('/pid_tuning_pitch', PidTune, self.x_set_pid)
        #rospy.Subscriber('/pid_tuning_roll', PidTune, self.y_set_pid)
        #rospy.Subscriber('/pid_tuning_throttle', PidTune, self.z_set_pid)
        #rospy.Subscriber('/edrone/gripper_check',String,self.check)
        rospy.Subscriber("qrcode",String,self.dst)
        rospy.Subscriber('/edrone/range_finder_top',LaserScan,self.top_callback)
        

        # SERVICES
        self.gp_srv = rospy.ServiceProxy('/edrone/activate_gripper',Gripper) # Creating connection to service
        self.gp_srv.wait_for_service() # Waiting for service to be running

    # Function to get current lattitude,longitude and altitude using gps

    def gps_callback(self,msg):

        self.current_coord[0] = msg.latitude
        self.current_coord[1] = msg.longitude
        self.current_coord[2] = msg.altitude


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

    #def check(self,msg):
        #self.near=msg.data

    # Function to activate gripper and it also returns True if box is attached to gripper

    def hold_box(self,value):
        req=GripperRequest()                  
        
        req.activate_gripper=value
               
        resp=self.gp_srv(req)  
               
           
        return resp.result

    # Function to get the coordinates of the destination from QR which also appends the coordinates in the global setpoints list

    def dst(self,msg):  
        if(self.c==1):
            txt=msg.data
            txt_lst=txt.split(",")
            lat=float(txt_lst[0])     
            longd=float(txt_lst[1]) 
            alt=float(txt_lst[2])
            
            r=0.00004517*4
            
            x=abs(self.setpoints[3][0]-lat)
            y=abs(self.setpoints[3][1]-longd)

            theta=math.atan(x/y) # Angle in radians

            d=((self.setpoints[3][0]-lat)**2+(self.setpoints[3][1]-longd)**2)**0.5

            n=int(d/r)

            if((lat-self.setpoints[3][0])>0):
                sign_x=1
            else:
                sign_x=-1
            if((longd-self.setpoints[3][1])>0):
                sign_y=1
            else:
                sign_y=-1

            if(self.setpoints[3][2]>alt):
                alti=self.setpoints[3][2]
            else:
                alti=alt


            for i in range(1,n+1):

                lati=self.setpoints[3][0]+r*(math.sin(theta))*sign_x*i
                longi=self.setpoints[3][1]+r*(math.cos(theta))*sign_y*i
                
                self.setpoints.append([lati,longi,alti])
            self.setpoints.append([lat,longd,alti])
            self.setpoints.append([lat,longd,alt])
            self.c=self.c+1
            print(self.setpoints)

    def top_callback(self,msg):
        self.lat_move=1
        self.lat_stay=1
        self.long_move=1
        self.long_stay=1

        left=msg.ranges[0]
        front=msg.ranges[1]
        right=msg.ranges[2]
        back=msg.ranges[3]

        if(back<=8 and back>=0.4):
            self.lat_move=0.0
            self.long_stay=0.0
                        
            self.obstacle_dist_x=back
            

        if(left<=6 and left>=0.4):
            self.lat_stay=0.0
            self.long_move=0.0
                       
            self.obstacle_dist_y=left

        if(self.r<=3 or self.r>=(len(self.setpoints)-2)):
            self.lat_move=1
            self.lat_stay=1
            self.long_move=1
            self.long_stay=1    

        
        self.x2=self.x1
        self.x1=self.lat_move

        self.y2=self.y1
        self.y1=self.long_move

        if((self.x1-self.x2==1)):
            self.esc_x=1

        if((self.y1-self.y2==1)):
            self.esc_y=1

        self.lat_move=1
        self.lat_stay=1
        self.long_move=1
        self.long_stay=1
  
           

    


    # Function to get pid value,and gives required orientation to attitude controller to traverse the required setpoints

    def pid(self,index_no):

        index_no=self.r # Passing the value of current index number so that there is no scope of error as sometimes None also gets passed

        now=time.time() # Getting the current time

        

        elapsed_time=now - self.lastTime # Time elapsed 
        
        
        
        if(elapsed_time>=0.001): # Sample time
            

            if(index_no!=len(self.setpoints) and index_no<len(self.setpoints)):
                # Calculating error in orientation
                
                self.current_error[0] = self.setpoints[index_no][0] - self.current_coord[0]
                self.current_error[1] = self.setpoints[index_no][1] - self.current_coord[1]
                self.current_error[2] = self.setpoints[index_no][2] - self.current_coord[2]
            
                # Storing the current error which will be published later

                #self.x_error.data = self.current_error[0]
                #self.y_error.data= self.current_error[0]
                #self.z_error.data = self.current_error[2]
            
                # Publishing the error

                #self.x_pub.publish(self.x_error)
                #self.y_pub.publish(self.y_error)
                #self.z_pub.publish(self.z_error)
                
                hold=self.hold_box(self.value) # To give value to activate/deactivate gripper,hold stores True/False if box is attached to gripper or not
                
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
                self.lastTime=now # Updating the value of last time

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

                # Setting the tolerance for lattitude,longitude and altitude
                # If the current coordinates is within the bounded range then lat/longt/alt is set to 1,else it is set to 0
##################################################################################################################            
                if(((self.setpoints[index_no][0]-0.000000717)<self.current_coord[0]) and ((self.setpoints[index_no][0]+0.000000717)>self.current_coord[0])):
                    lat=1
                else:
                    lat=0 
           
                if(((self.setpoints[index_no][1]-0.0000007487)<self.current_coord[1]) and ((self.setpoints[index_no][1]+0.0000007487)>self.current_coord[1])):
                    longt=1
                else:
                    longt=0  

                if(((self.setpoints[index_no][2]-0.05)<self.current_coord[2]) and ((self.setpoints[index_no][2]+0.05)>self.current_coord[2])):
                    alt=1
                else:
                    alt=0 
###############################################################################################################
                # If first setpoint is reached then increment the index so that next setpoint is selected,else it is not incremented and remains same                
                if(lat+longt+alt==3 and (index_no==0 or index_no==1)):
                    index_no=index_no+1
                    self.value = True
                    lat=0 # lat is intentionally set to 0 so that the condition lat+longt+alt is not satisfied and index is not incremented for the below 'if' as self.value is set True
                    
                # This is used after the first setpoint is reached
                if(lat+longt+alt==3): #and hold==True):
                    index_no=index_no+1
                    
                else:
                    index_no=index_no
                
                # This is used after last setpoint is reached
                if(lat+longt+alt==3 and index_no==len(self.setpoints)):
                    index_no=index_no-1
                    self.value = False

                
            
            # If index becomes out of range then all setpoints have been reached and we need to switch off the drone             

            if(index_no>=len(self.setpoints)):
                index_no=len(self.setpoints)-1
                #self.drone_orientation.rcRoll = 1500
                #self.drone_orientation.rcPitch = 1500
                #self.drone_orientation.rcYaw = 1500
                #self.drone_orientation.rcThrottle = 1000
                #self.rc_pub.publish(self.drone_orientation)

            self.r=index_no # Storing the current index number
                

            return index_no
                           
    def pid_local(self):
        
        if(self.change==1):
            self.setpoints_local=[]
            lati=self.current_coord[0] + ((7.5-self.obstacle_dist_x)/110692.0702932625)*self.lat_stay -(5/110692.0702932625)*self.lat_move
            longi=self.current_coord[1] + ((4-self.obstacle_dist_y)/105292.0089353767)*self.long_stay +(5/105292.0089353767)*self.long_move                  
            alti=self.setpoints[3][2]

            self.setpoints_local.append([lati,longi,alti])
            print(self.setpoints_local)
        
        if(self.change==1):
            if((self.lat_move+self.long_stay)==0 and (abs(self.current_coord[0]-self.setpoints[self.r][0])<=(8/110692.0702932625))):
                if((longi>=self.setpoints[self.r][1]) and (self.r!=(len(self.setpoints)-2))):
                    self.r=self.r+1
                    print(self.r)
            if(self.lat_stay+self.long_move==0): 
                if((lati<=self.setpoints[self.r][0]) and (self.r!=(len(self.setpoints)-2))):
                    self.r=self.r+1        
                    print(self.r)
        now=time.time() # Getting the current time

        

        elapsed_time=now - self.lastTime # Time elapsed 
        
        
        
        if(elapsed_time>=0.001 and len(self.setpoints_local)==1): # Sample time
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
            self.lastTime=now # Updating the value of last time

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

            
    def pid_local_escape(self):
        
        self.change=1

        if(self.esc_x==1):
            self.setpoints_local=[]
            
            lati=self.current_coord[0] -(8/110692.0702932625) #5
            longi=self.current_coord[1]  +(3/105292.0089353767)                 
            alti=self.setpoints[3][2]
            self.setpoints_local.append([lati,longi,alti])
             
               

        if(self.esc_y==1):
            self.setpoints_local=[]
            
            lati=self.current_coord[0] -(8/110692.0702932625)
            longi=self.current_coord[1]  -(2/105292.0089353767)                 
            alti=self.setpoints[4][2]
            self.setpoints_local.append([lati,longi,alti])
             
               
        
        if(self.esc_x==1):
            if(longi>=self.setpoints[self.r][1] and (self.r!=(len(self.setpoints)-2)) and (abs(self.current_coord[0]-self.setpoints[self.r][0])<=(8/110692.0702932625))):
                self.r=self.r+1        
                self.esc_x=0
                print(self.r)
                print("escx")
            else:
                self.esc_x=0

        if(self.esc_y==1):
            if(lati<=self.setpoints[self.r][0] and (self.r!=(len(self.setpoints)-2))):
                self.r=self.r+1        
                self.esc_y=0
                print(self.r)
                print("escy")
            else:
                self.esc_y=0

        now=time.time() # Getting the current time

        

        elapsed_time=now - self.lastTime # Time elapsed 
        
        
        
        if(elapsed_time>=0.001 and len(self.setpoints_local)==1): # Sample time
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
            self.lastTime=now # Updating the value of last time

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
           

                
if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_frequency)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    rospy.loginfo("started")
    i=0 # To pass index for setpoints
    while not rospy.is_shutdown():
        if((e_drone.lat_move + e_drone.lat_stay + e_drone.long_move + e_drone.long_stay==4) and (len(e_drone.setpoints_local)==0)):               
            i=e_drone.pid(i) # Current index number is passed and is returned after it is updated
        if((e_drone.lat_move + e_drone.lat_stay + e_drone.long_move + e_drone.long_stay==2) ):
            e_drone.pid_local() 

        if(e_drone.lat_move + e_drone.lat_stay + e_drone.long_move + e_drone.long_stay==4) and ((len(e_drone.setpoints_local)!=0)):
            e_drone.pid_local_escape()
             
        #if(i>=len(e_drone.setpoints)): # If index number becomes out of range then break
            #break

        r.sleep()             


     


