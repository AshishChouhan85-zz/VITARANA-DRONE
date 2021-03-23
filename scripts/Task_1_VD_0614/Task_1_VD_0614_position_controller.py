#!/usr/bin/env python




# Importing the required libraries

from vitarana_drone.msg import edrone_cmd
from std_msgs.msg import Float32
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
import math
import rospy
import time


class Edrone():
    
    def __init__(self):
        rospy.init_node('position_controller') # initializing ros node with name position_control

        # Coordinates of setpoints stored in a list
        self.setpoints = [[19.0,72.0,0.31],[19.0,72.0,3.0],[19.0000451704,72.0,3.0],[19.0000451704,72.0,0.31]]
       
        self.lastTime=0.0 # To store the last time

        self.r=0 # To store value of current index number 

        # List for storing current [latitude,longitude,altitude]
        self.current_coord = [0.0,0.0,0.0]

        # Variables for storing error in latitude , longitude and altitude
        #self.x_error = Float32()
        #self.y_error = Float32()
        #self.z_error = Float32()

        #self.x_error.data = 0.0
        #self.y_error.data = 0.0
        #self.z_error.data = 0.0

        # Declaring drone_orientation of message type edrone_cmd and initialising values

        self.drone_orientation = edrone_cmd()

        # initial setting of Kp, Kd and ki for [latitude, longitude, altitude]

        self.Kp = [2770000, 3600000, 120.1]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [25876700, 38500000, 303.5]

        # Initialising lists to store errors
        
        self.current_error=[0, 0, 0]       # To store current error in [latitude,longitude,altitude] format
        self.previous_error=[0, 0, 0]      # To store previous error in [latitude,longitude,altitude] format
    
        # Storing max and min values for pwm

        self.max_values = [2000,2000,2000,2000] # Max limit corresponding to [rcRoll,rcPitch,rcYaw,rcThrottle]
        self.min_values = [1000,1000,1000,1000] # Min limit corresponding to [rcRoll,rcPitch,rcYaw,rcThrottle]
        
        # Proportional,Integral and Derivative values in [latitude,longitude,altitude] format

        self.P = [0, 0, 0]
        self.I = [0, 0, 0]   
        self.D = [0, 0, 0]     

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

    # Function to get current lattitude,longitude and altitude using gps

    def gps_callback(self,msg):

        self.current_coord[0] = msg.latitude
        self.current_coord[1] = msg.longitude
        self.current_coord[2] = msg.altitude


    #def x_set_pid(self, x):
        #self.Kp[0] = x.Kp*10000  
        #self.Ki[0] = x.Ki*1000 
        #self.Kd[0] = x.Kd*10000 


    #def y_set_pid(self, y):
        #self.Kp[1] = y.Kp*10000  
        #self.Ki[1] = y.Ki*1000 
        #self.Kd[1] = y.Kd*10000 
 
    
    #def z_set_pid(self, z):
        #self.Kp[2] = z.Kp*0.1  
        #self.Ki[2] = z.Ki*0.01
        #self.Kd[2] = z.Kd*0.1  


    # Function to get pid value,and gives required orientation to attitude controller to traverse the required setpoints

    def pid(self,index_no):

        index_no=self.r # Passing the value of current index number so that there is no scope of error as sometimes None also gets passed

        now=time.time() # Getting the current error

        

        elapsed_time=now - self.lastTime # Time elapsed 
        
        
        
        if(elapsed_time>=0.001): # Sample time
            

            if(index_no!=len(self.setpoints)):
                # Calculating error in orientation
                
                self.current_error[0] = self.setpoints[index_no][0] - self.current_coord[0]
                self.current_error[1] = self.setpoints[index_no][1] - self.current_coord[1]
                self.current_error[2] = self.setpoints[index_no][2] - self.current_coord[2]
            
                # Storing the current error which will be published later

                #self.x_error.data = self.current_error[0]
                #self.y_error.data= self.current_error[1]
                #self.z_error.data = self.current_error[2]
            
                # Publishing the error

                #self.x_pub.publish(self.x_error)
                #self.y_pub.publish(self.y_error)
                #self.z_pub.publish(self.z_error)
               

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
                self.I[2]=self.I[2] + self.Ki[2]*self.current_error[2]*elapsed_time
                self.D[2]=self.Kd[2]*(self.current_error[2]-self.previous_error[2])/elapsed_time
            
                self.out_alt=self.P[2] + self.I[2] + self.D[2]

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
            
                if(((self.setpoints[index_no][0]-0.000002517)<self.current_coord[0]) and ((self.setpoints[index_no][0]+0.000002517)>self.current_coord[0])):
                    lat=1
                else:
                    lat=0 
           
                if(((self.setpoints[index_no][1]-0.0000027487)<self.current_coord[1]) and ((self.setpoints[index_no][1]+0.0000027487)>self.current_coord[1])):
                    longt=1
                else:
                    longt=0  

                if(((self.setpoints[index_no][2]-0.05)<self.current_coord[2]) and ((self.setpoints[index_no][2]+0.05)>self.current_coord[2])):
                    alt=1
                else:
                    alt=0 

                # If one setpoint is reached then increment the index so that next setpoint is selected,else it is not incremented and remains same                

                if(lat+longt+alt==3):
                    index_no=index_no+1
                else:
                    index_no=index_no
            
            # If index becomes out of range then all setpoints have been reached and we need to switch off the drone             

            if(index_no==len(self.setpoints)):
                self.drone_orientation.rcRoll = 1500
                self.drone_orientation.rcPitch = 1500
                self.drone_orientation.rcYaw = 1500
                self.drone_orientation.rcThrottle = 1000
                self.rc_pub.publish(self.drone_orientation)

            self.r=index_no # Storing the current index number
                

            return index_no
                           

                          


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_frequency)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    rospy.loginfo("started")
    i=0 # To pass index for setpoints
    while not rospy.is_shutdown():
                
        i=e_drone.pid(i) # Current index number is passed and is returned after it is updated

        if(i==len(e_drone.setpoints)): # If index number becomes out of range then break
            break

        r.sleep()             


     


