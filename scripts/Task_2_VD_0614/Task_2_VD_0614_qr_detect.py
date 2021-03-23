#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
from pyzbar import pyzbar
from std_msgs.msg import String

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode') #Initialise rosnode 

                self.coord=String() 
                self.coord.data="No QR detected"                

		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
                self.image_pub = rospy.Publisher("qrcode", String, queue_size=10) #Publishing 
		self.bridge = CvBridge() 


	# Callback function of amera topic
	def image_callback(self, data):
                #cv2.namedWindow("j")
		try:
			cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
                 
               	except CvBridgeError as e:
			print(e)
	        barcodes=[]
                barcodes=pyzbar.decode(cv_img) # Gives info of decoded data and coordinates of QR 

                for barcode in barcodes:
                    bdata=barcode.data.decode('utf-8') # Converting bytes into string
                    self.coord.data=bdata
                #cv2.imshow("j",cv_img)
                #k=cv2.waitKey(1)               

                if(len(barcodes)!=0):
                    self.image_pub.publish(self.coord)
                    
if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.loginfo("started")
    rospy.spin()
