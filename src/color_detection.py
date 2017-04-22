#!/usr/bin/env python

import rospy
import random
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
class color_shape_detector:

	def __init__(self):
		self.bridge = CvBridge()
		self.success_color = False
		self.success_shape = False
		self.election_color = ''
		self.election_shape = ''
		#creating a filter
		#Position 0 is the lower limit and positon 1 the upper one
		blue_threshold = np.array([[103,50,50],[130,255,255]])
		red_threshold = np.array([[169, 100, 100],[189, 255, 255]])
		green_threshold = np.array([[49,50,50],[80, 255, 255]])
		self.colors = {'Blue': blue_threshold, 'Red': red_threshold, 'Green': green_threshold}
		self.shapes = {'Triangle' : 3, 'Square' : 4, "Circle" : 15}
		self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.confirmation)

	def set_random_parameters(self):
		self.election_color = random.choice(self.colors.keys())
		self.election_shape = random.choice(self.shapes.keys())

	def reset_parameters(self):
		self.success_color = False
		self.success_shape = False

	def confirmation(self,ros_image):	
		#convertion from ROS Image format to opencv and filtering
		inImg = self.bridge.imgmsg_to_cv2(ros_image,"bgr8")
		inImg_filtered = cv2.GaussianBlur(inImg, (5,5),0)
		
		#convertion from rgb to hsv
		inImg_hsv = cv2.cvtColor(inImg_filtered, cv2.COLOR_BGR2HSV)

		#appliying the color filter
		mask = cv2.inRange(inImg_hsv,self.colors[self.election_color][0],self.colors[self.election_color][1])
		cv2.imshow("mask", mask)
		cv2.waitKey(1)

		#morphological transformation
		kernel = np.ones((5,5),np.uint8)
		mask_op = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
		mask_op_cl = cv2.morphologyEx(mask_op, cv2.MORPH_CLOSE,kernel)
		cv2.imshow("mask opening closing", mask_op_cl)
		moments = cv2.moments(mask_op_cl)
		area = moments['m00']
		if area > 2000000:
		 	self.success_color = True
		
		#appliying the shape filter
		#ret,thresh_shape = cv2.threshold(inImg_gray,127,255,1)
		ret,thresh_shape = cv2.threshold(mask_op_cl,127,255,1)
		contours,h = cv2.findContours(thresh_shape,1,2)

		for cnt in contours:
			approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
    		if len(approx) == self.shapes[self.election_shape]:
    			self.success_shape = True
    			print "ok"
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

	cd = color_shape_detector()
	cd.set_random_parameters()
	
	rospy.init_node('color_reciever', anonymous = 'True' )
	while not rospy.is_shutdown():
		print  "Show me the " +  cd.election_color  + " " + cd.election_shape 
		while (cd.success_color == False and cd.success_shape == False and not rospy.is_shutdown()):
			pass
		print "exited"
		cd.reset_parameters()
		cd.set_random_parameters()
	cv2.destroyAllWindows()

if __name__ == '__main__':
    color_detection()
    rospy.spin()
	