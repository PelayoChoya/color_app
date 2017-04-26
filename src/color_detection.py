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
		#cv2.imshow("mask", mask)
		#cv2.waitKey(1)

		#morphological transformation
		kernel = np.ones((7,7),np.uint8)
		mask_op = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
		mask_op_cl = cv2.morphologyEx(mask_op, cv2.MORPH_CLOSE,kernel)
		#cv2.imshow("mask opening closing", mask_op_cl)


		#removing the small objects from the binary image
		contours,h = cv2.findContours(mask_op_cl,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		mask_hue = np.ones(mask_op_cl.shape[:2], dtype="uint8") * 255
		#cv2.imshow("middle step", mask_hue)
		area_ev = 0
		iterator = 0
		biggest_area_index = 0
		
		if contours:
			
			for cnt in contours:
				area = cv2.contourArea(cnt)
				if (area > area_ev):
					area_ev = area
					biggest_area_index = iterator
				iterator = iterator + 1

			cnt =  contours[biggest_area_index]
			cv2.drawContours(mask_hue, [cnt], -1, 0, -1)
			cv2.bitwise_not(mask_hue,mask_hue)
			cv2.imshow("hue", mask_hue)
			cv2.waitKey(1)
			#check if the color filer succeed
			if area_ev > 20000:
			 	self.success_color = True
			
			#appliying the shape filter
			if(self.election_shape == 'Circle') :
				circles = cv2.HoughCircles(mask_hue,cv2.cv.CV_HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=100,maxRadius=0)
				if circles is not None:
					self.success_shape = True
			else :
				pass
		    	if len(cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)) == self.shapes[self.election_shape]:
		    		self.success_shape = True
		else:
			pass
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
	