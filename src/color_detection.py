#!/usr/bin/env python

import rospy
import random
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
class color_detector:

	def __init__(self):
		self.bridge = CvBridge()
		self.success = False
		self.election = ''
		#creating a filter
		#Position 0 is the lower limit and positon 1 the upper one
		blue_threshold = np.array([[110,50,50],[130,255,255]])
		red_threshold = np.array([[169, 100, 100],[189, 255, 255]])
		green_threshold = np.array([[49,50,50],[80, 255, 255]])
		self.colors = {'Blue': blue_threshold, 'Red': red_threshold, 'Green': green_threshold}
		self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.confirmation)

	def set_random_color(self):
		self.election = random.choice(self.colors.keys())

	def reset_parameters(self):
		self.success = False

	def confirmation(self,ros_image):	
		#convertion from ROS Image format to opencv
		inImg = self.bridge.imgmsg_to_cv2(ros_image,"bgr8")
		cv2.imshow("Raw", inImg)
		

		#convertion from rgb to hsv
		inImg_hsv = cv2.cvtColor(inImg, cv2.COLOR_BGR2HSV)
		cv2.imshow("camera", inImg_hsv)  

		#appliying the filter
		mask = cv2.inRange(inImg_hsv,self.colors[self.election][0],self.colors[self.election][1])
		cv2.imshow("mask", mask)
		cv2.waitKey(1)
		moments = cv2.moments(mask)
		area = moments['m00']
		if area > 2000000:
		 	self.success = True
		# 		print "yes"
		
		# #Noise elimination
		# moments = cv2.moments(mask)
		# area = moments['m00']
		# if (area > 200000):
		# 	#Looking for the centers
		# 	x = int(moments['m10']/moments['m00'])
	 #   		y = int(moments['m01']/moments['m00'])	

	 #   		#Center drawing
	 #   		cv2.rectangle(inImg, (x, y), (x+2, y+2),(0,0,255), 2)
	 #   		cv2.imshow('final',inImg)
	 #    	#displaying the images
	 #        cv2.waitKey(1)

def  color_detection():

	cd = color_detector()
	cd.set_random_color()
	
	rospy.init_node('color_reciever', anonymous = 'True' )
	while not rospy.is_shutdown():
		print  "Show me the " + cd.election + "Color"
		while (cd.success == False and not rospy.is_shutdown()):
			pass
		print "exited"
		cd.reset_parameters()
		cd.set_random_color()
	rospy.spin()
	cv2.destroyAllWindows()

if __name__ == '__main__':
    color_detection()
	