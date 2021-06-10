# INTRODUCTION
In urban streets, especially South Asian streets, clogged with automobiles, people, animals, markets and numerous other things ! using typical modes of transport for delivery, such as cars and bikes, is slow and wasteful. Fortunately, enabled by progress in embedded systems, materials science and control systems, unmanned aerial vehicles (UAVs), have become widespread in the past couple of decades and provide a simpler and faster alternative to using terrestrial infrastructure!<br>

These UAVs are a result of improvements in semiconductor and microprocessor technology, enabling faster chips and smaller, more efficient, power electronics. Improvements in materials sciences & manufacturing have led to the creation of novel, cheap, precise & robust sensors, along with huge improvements in battery technology. There have been wide-ranging developments in the domain of control systems powering the algorithms that run onboard as well as the techniques that power these motors. All of this has created a virtuous feedback loop of improvements, driven by widespread commercialization in both hobby and industrial spheres. UAVs now range from quadcopters smaller than a child’s palm, to large airplanes that can fly for days, while being commanded from thousands of kilometres away.<br>

This project works on concepts of <b>control systems, path planning, image processing and algorithm development.</b><br>
Tools such as the <b>Robot Operating System, robotics simulator Gazebo, the Python programming language</b> and many of its libraries are used.<br>

The competition consists of series of tasks, the final problem statement for the drone is to deliver various packages to their destinations, optimizing for time and quantity.<br>

# TASK 0
## Problem Statement
The objective of the task is to move the turtle inside the turtlesim window in a circle and stop at its initial location.
Teams are supposed to do this by creating a nodes name, /node_turtle_revolve within a python script, node_turtle_revolve.py.<br>

![task0](https://github.com/AshishChouhan85/VITARANA-DRONE/blob/main/vitarana_drone/scripts/Task_0_VD_0614/VD_0614.png)<br>
This task was given to make us familiar with ROS and its basics.<br>

# TASK 1
## Problem Statement
The aim of this task is to design controllers which will control the eDrone's orientation as well as position in Gazebo environment.<br>
The Task 1 is divided into 2 sub tasks<br>
- Task 1A - Designing attitude controller for the eDrone<br>
- Task 1B - Designing position controller for the eDrone<br>

## Position Controller
The main task of position controller is to give the required drone orientation to reach to the required setpoint. The required orientation is calculated using a PID controlled algorithm.The orientation is published in quaternion format which ranges from 1000 to 2000. 1000 corresponds to -10 degrees and 2000 corresponds to 10 degress and all the angles between -10 and 10 degrees can be found accordingly.



## Attitude Controller
The main task of attitude controller is to keep the drone in the required orientation which is given by position controller. This is done by another PID controlled algorithm. After calculating the required PID values throttle is given to each motor of the drone so that it flies in the required orientation.These values are then converted to required PWM format that could be given to motors.<br>
<b>NOTE: The signs infront of pitch,roll and yaw is given according to the structure of the drone,these may vary from one drone to other.</b>

```python
# Giving throttle to each motor in 1000-2000 format

motor1 = self.setpoint_throttle + self.out_roll - self.out_pitch - self.out_yaw
motor2 = self.setpoint_throttle - self.out_roll - self.out_pitch + self.out_yaw
motor3 = self.setpoint_throttle - self.out_roll + self.out_pitch - self.out_yaw
motor4 = self.setpoint_throttle + self.out_roll + self.out_pitch + self.out_yaw

```
In total 2 PID algorithms were applied to control the flight of the drone. The diagram of the cascaded control system is shown below,
<br>
![Cascaded Control System](https://user-images.githubusercontent.com/60431758/121469207-b5327080-c9d9-11eb-812b-1308918b8410.png)


The path of the drone was hardcoded for this task giving it 3 coordinates to reach the destination. The final video of task 1 is shown below,<br>

![Task1](https://user-images.githubusercontent.com/60431758/112194024-7983d800-8c2e-11eb-95e3-cb833fc3441a.gif)<br>

# TASK 2
## Problem Statement
The aim of this task is to pick a parcel and deliver it to its destination<br>
The Task 2 is divided into 3 sub tasks<br>
- Task 2A - Scanning the QR code and finding out the destination GPS co-ordinates.
- Task 2B - Pick/Drop the parcel box
- Task 2C - Avoiding dynamic obstacles and planing the path

## SCANNING THE QR
The QR Code is found using pyzbar module of python. The data received is the gps coordinates of the final location and is sent to position controller through a publisher. There is a minimum height at which camera can detect the QR code correctly.

## PICK/DROP THE PARCEL BOX
The box can be picked/dropped using the a rosservice named ```/edrone/activate_gripper```. For picking the box, eDrone has to land exactly on top of the box on the centre and then the gripper is activated.

![ezgif com-gif-maker (5)](https://user-images.githubusercontent.com/60431758/112269429-5e50b100-8c9e-11eb-87e1-e4b40600aa2a.gif)

## AVOIDING DYNAMIC OBSTACLES
Unlike the previous task the path of the drone can be hardcoded only until it picks the box. After that drone has to detect and avoid obstacles on its own. Obstacles can be detected using four distance sensors loaded on four sides of the drone. After that some dynamically calculated setpoints are given to drone to avoid the obstacle. As the map of area is not given beforehand path planning algorithms like Dijkstra's algorithm cannot be used. Here we have used bug algorithm that does not require any map of the area it is about to enter.Also small setpoints were given to drone to reach it final setpoint, these setpoints were dynamically created. This algo was used in Task 4 as well but dropped after that as it was reducing the speed of the drone.

![ezgif com-gif-maker (6)](https://user-images.githubusercontent.com/60431758/112269464-690b4600-8c9e-11eb-9037-5a340d751bb4.gif)

# TASK 3
## Problem Statement
The aim of this task is to detect the landing markers present in the general vicinity of a given GPS coordinate. The GPS coordinate of the landing marker will NOT be given, instead the coordinate will point somewhere around the marker. You have to hover at certain height from the given coordinates and scan for the landing marker by applying image processing techniques and design an algorithm to take the eDrone above the landing marker and eventually land on it.

For getting a view of the area below drone a camera was attached under it. Frames were captured and image processing was done to detect the marker. A LBP Cascade classifier was trained using 34320 positive samples and 34320 negative samples to detect the marker. We also needed to calculate the distance of drone from centre of marker in metres. For this we used the formula, ```centre_x_pixel*Z_m/focal_length``` ,here Z_m is calculated using distance sensor which is attached below the drone and centre_x_pixel is the pixel coordinate of centre of marker along x axis. This formula gives distance in metres. focal_length is calculated using ```focal_length = (img_width/2)/tan(hfov_rad/2)``` where img_width=400 and hfov_rad stands for the horizontal field of view of the camera in radians, in our case it is 1.3962634. Also there was some fixed error in the values we get from this formula,hence it was subtracted and then given to drone as a setpoint to reach the centre of marker. 

```python
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
```

![ezgif com-gif-maker (3)](https://user-images.githubusercontent.com/60431758/112222968-430a8500-8c4f-11eb-9242-25e480978fff.gif)

# TASK 4
## Problem Statement
The aim of this task is to complete a set of deliveries from warehouse to their destinations avoiding obstacles.

In this task we just edited our codes a little bit such that it picks and drops the parcel at right setpoints. Also we made a new function which plans a path of the drone by giving small setpoints as it was done in Task 2. Giving small setpoints make obstacle avoidance easier as drone had enough time to detect and avoid the obstacle. This function gets activated whenever the distance between two setpoints is more than 30 metres.

```python
def path_planner(self,index_no): 

    self.index_list=[] # Emptying the list to give new coordinates for each run

    r=25  # This is the distance of between dynamically added setpoints
            
    x=(abs(self.setpoints[index_no][0]-self.setpoints[index_no-1][0]))*110692.0702932625 # Calculating total latitude distance between current setpoint and destination setpoint in metres
    y=(abs(self.setpoints[index_no][1]-self.setpoints[index_no-1][1]))*105292.0089353767 # Calculating total longitude distance between current setpoint and destination setpoint in metres
    theta=math.atan(x/y) # Angle in radians,which is always positive     
         

    d=self.path_dist # Storing total distance between current setpoint and destination setpoint

    n=int(d/r) # Getting required number of setpoints to reach close to destination

    # Saving sign of destination setpoint minus current setpoint

    if((self.setpoints[index_no][0]-self.setpoints[index_no-1][0])>0):
        self.sign_x=1
    else:
        self.sign_x=-1         

    if((self.setpoints[index_no][1]-self.setpoints[index_no-1][1])>0):
        self.sign_y=1
    else:
        self.sign_y=-1

    alti=self.setpoints[index_no-1][2]

        
    # Giving setpoints
     
    for i in range(1,n+1):

        lati=self.setpoints[index_no-1][0]+(r*(math.sin(theta))*self.sign_x)/110692.0702932625
        longi=self.setpoints[index_no-1][1]+(r*(math.cos(theta))*self.sign_y)/105292.0089353767
                
        self.setpoints.insert(index_no,[lati,longi,alti,1])
        self.index_list.append(index_no)
        index_no=index_no+1

    # Storing whether latitude is larger or longitude is larger as it is helpful in obstacle avoidance  
    
    if(x>y):
        self.path="X"
    else:
        self.path="Y"
```            

![ezgif com-gif-maker (4)](https://user-images.githubusercontent.com/60431758/112224232-050e6080-8c51-11eb-9005-a56edd9668a2.gif)

# TASK 5 and 6
## Problem Statement
The aim of both tasks was to write a scheduler algorithm to plan the order of jobs (delivery or return) such that the the earnings of eDrone is maximum in the given time limit.

In these tasks we have to first deliver a parcel to its destination location and then search for a return parcel nearby and bring it back to warehouse. We cannot hardcode any coordinates in this task. Everthing was decided by the scheduler algorithm before the drone takes off. Our scheduler algorithm sorts the distance of delivery locations in descending order. This was our stratergy to gain maximum earning.
