#!/usr/bin/env python

import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 

def confirmation(ros_image):	
	#convertion from ROS Image format to opencv
	bridge = CvBridge()
	inImg = bridge.imgmsg_to_cv2(ros_image,"bgr8")

	#convertion from rgb to hsv
	inImg_hsv = cv2.cvtColor(inImg, cv2.COLOR_BGR2HSV)
	cv2.imshow("camera", inImg_hsv)  

	#creating a filter
	#Position 0 is the lower limit and positon 1 the upper one
	blue_threshold = np.array([[110,50,50],[130,255,255]])
	red_threshold = np.array([[169, 100, 100],[189, 255, 255]])
	green_threshold = np.array([[49,50,50],[80, 255, 255]])
	colors = {'Blue': blue_threshold, 'Red': red_threshold, 'Green': green_threshold}
	mask = cv2.inRange(inImg_hsv,colors['Green'][0],colors['Green'][1])
	cv2.imshow("mask", mask)
	#Noise elimination
	moments = cv2.moments(mask)
	area = moments['m00']
	if (area > 200000):
		#Looking for the centers
		x = int(moments['m10']/moments['m00'])
   		y = int(moments['m01']/moments['m00'])	

   		#Center drawing
   		cv2.rectangle(inImg, (x, y), (x+2, y+2),(0,0,255), 2)
   		cv2.imshow('final',inImg)
    	#displaying the images
        cv2.waitKey(1)

def  color_detection():
	rospy.init_node('color_reciever', anonymous = 'True' )
	rospy.Subscriber("/camera/rgb/image_color", Image, confirmation)
	rospy.spin()

if __name__ == '__main__':
    color_detection()
	