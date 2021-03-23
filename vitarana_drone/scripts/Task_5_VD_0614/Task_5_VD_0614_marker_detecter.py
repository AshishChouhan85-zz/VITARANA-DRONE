#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import math
from std_msgs.msg import String



class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('detect_marker') #Initialise rosnode 
                self.logo_cascade = cv2.CascadeClassifier('/home/utkarsh/catkin_ws/src/vitarana_drone/scripts/Task_3_VD_0614_cascade.xml')

                 
                              
                self.coord=String()
               
                self.coord.data="NO MARKER"
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
                
                self.coord_pub = rospy.Publisher("pixel_coord", String, queue_size=10)
		self.bridge = CvBridge() 


	# Callback function of amera topic
	def image_callback(self, data):
		try:
			cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
                 
               	except CvBridgeError as e:
			print(e)

                
                gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
                logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
                self.coord.data="NO MARKER"
                for (x, y, w, h) in logo:
                    if(w>=27 and w<=34):
                        cv_img=cv2.rectangle(cv_img,(x,y),(x+w,y+h),(0,0,255),5)
                        X=(x+x+w)//2
                        Y=(y+y+h)//2
                        self.coord.data=str(X)+","+str(Y)
                    
                    else:
                        self.coord.data="NO MARKER"    
                    
                cv2.imshow("j",cv_img)
                cv2.waitKey(1)    
                self.coord_pub.publish(self.coord)
                    
if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.loginfo("started")
    
    rospy.spin()




