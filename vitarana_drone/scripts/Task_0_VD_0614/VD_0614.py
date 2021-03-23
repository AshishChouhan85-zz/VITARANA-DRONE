#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


X=0
Y=0
Xi=0
Yi=0
dist=0
count=0

### FUNCTION TO GET CURRENT COORDINATES OF THE TURTLE AND TO GET ITS DISTANCE COVERED SO FAR  ####

def pose_callback(pose):   
    global X,Y,count,Xi,Yi,dist
    
    if(count!=0):
        Xi=X
        Yi=Y
    

    X=pose.x
    Y=pose.y

    if(count==0):
        Xi=X
        Yi=Y
    ds=math.sqrt((X-Xi)**2+(Y-Yi)**2)  # CALCULATING SMALL DISTANCES
    dist=dist+ds                       # ADDING ALL THE SMALL DISTANCES  
                                       # NOTE:CALCULATING THE DISTANCE IS NECESSARY HERE AS SOME POINTS MIGHT BE SKIPPED IF DONE OUTSIDE THIS FUNCTION
    count+=1                           

   

rospy.init_node("node_turtle_revolve")                # NAMING THE NODE
pub=rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=1)#MAKING A PUBLISHER AND GIVING IT A TOPIC
rospy.Subscriber("/turtle1/pose",Pose,pose_callback)# MAKING A SUBSCRIBER FOR A TOPIC

rate=rospy.Rate(10) # 10 HZ FREQUENCY

move=Twist() # INSTANCE OF TWIST

move.linear.x=3 # SETTING LINEAR VELOCITY TO 3m/s
move.angular.z=1.5 # SETTING ANGULAR VELOCITY TO 1.5rad/s

radius=move.linear.x/move.angular.z # CALCULATING RADIUS OF THE CIRCLE

circum=2*(math.pi)*radius  # CALCULATING CIRCUMFERENCE OF THE CIRCLE

rospy.loginfo("start")


while not rospy.is_shutdown():
    
    
    
    if(dist<circum):        # IF TOTAL DISTANCE COVERED IS LESS THAN CIRCUMFERENCE
        pub.publish(move)
        rospy.loginfo("Moving in a circle:")
        rate.sleep()
    
    else:
        move.linear.x=0
        move.angular.z=0
        pub.publish(move)
        rospy.loginfo("reached goal")
        break
    
    
