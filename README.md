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
The main task of attitude controller is to keep the drone in the required orientation which is given by position controller. This is done by another PID controlled algorithm which finally calculates the required PWM values. After calculating the required PID values throttle is given to each motor of the drone so that if flies in the required orientation.These values are then converted to required PWM format that could be given to motors.<br>
<b>NOTE: The signs infront of pitch,roll and yaw is given according to the structure of the drone,these may vary from one drone to other.</b>

```python
# Giving throttle to each motor in 1000-2000 format

motor1 = self.setpoint_throttle + self.out_roll - self.out_pitch - self.out_yaw
motor2 = self.setpoint_throttle - self.out_roll - self.out_pitch + self.out_yaw
motor3 = self.setpoint_throttle - self.out_roll + self.out_pitch - self.out_yaw
motor4 = self.setpoint_throttle + self.out_roll + self.out_pitch + self.out_yaw

```
In total 2 PID algorithms were applied to control the flight of the drone. The diagram of the cascaded control system is shown below,
<p align="center">
![task1](https://github.com/AshishChouhan85/VITARANA-DRONE/blob/main/Images/Cascaded%20Control%20System.png)<br>
</p>
The path of the drone was hardcoded for this task. The final video of task 1 is shown below,

![Task1](https://user-images.githubusercontent.com/60431758/112194024-7983d800-8c2e-11eb-95e3-cb833fc3441a.gif)<br>






