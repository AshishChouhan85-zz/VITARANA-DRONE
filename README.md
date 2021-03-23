# INTRODUCTION
In urban streets, especially South Asian streets, clogged with automobiles, people, animals, markets and numerous other things(!) using typical modes of transport for delivery, such as cars and bikes, is slow and wasteful. Fortunately, enabled by progress in embedded systems, materials science and control systems, unmanned aerial vehicles (UAVs), have become widespread in the past couple of decades and provide a simpler and faster alternative to using terrestrial infrastructure!<br>

These UAVs are a result of improvements in semiconductor and microprocessor technology, enabling faster chips and smaller, more efficient, power electronics. Improvements in materials sciences & manufacturing have led to the creation of novel, cheap, precise & robust sensors, along with huge improvements in battery technology. There have been wide-ranging developments in the domain of control systems powering the algorithms that run onboard as well as the techniques that power these motors. All of this has created a virtuous feedback loop of improvements, driven by widespread commercialization in both hobby and industrial spheres. UAVs now range from quadcopters smaller than a child’s palm, to large airplanes that can fly for days, while being commanded from thousands of kilometres away.<br>

This project works on concepts of <b>control systems, path planning, image processing and algorithm development.</b><br>
Tools such as the <b>Robot Operating System, robotics simulator Gazebo, the Python programming language</b> and many of its libraries are used.<br>

The competition consists of series of tasks, the final problem statement for the drone is to deliver various packages to their destinations, optimizing for time and quantity.<br>

# TASK 0
## Problem Statement
The objective of the task is to move the turtle inside the turtlesim window in a circle and stop at its initial location.
Teams are supposed to do this by creating a nodes name, /node_turtle_revolve within a python script, node_turtle_revolve.py.<br>

![task0](https://github.com/AshishChouhan85/VITARANA-DRONE/blob/main/scripts/Task_0_VD_0614/VD_0614.png)<br>
This task was given to make us familiar with ROS and its basics.<br>

# TASK 1
## Problem Statement
The aim of this task is to design controllers which will control the eDrone's orientation as well as position in Gazebo environment.<br>
The Task 1 is divided into 2 sub tasks<br>
- Task 1A - Designing attitude controller for the eDrone<br>
- Task 1B - Designing position controller for the eDrone<br>

## Position Controller
The main task of position controller is to give the required drone orientation to reach to the required setpoint. The orientation is given in quaternion format which ranges from 1000 to 2000. 1000 corresponds to -10 degrees and 2000 corresponds to 10 degress and all the angles between -10 and 10 degrees can be found accordingly. The required orientation is calculated using a PID controlled algorithm as shown below,<br>

'''python

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
'''               
 
