#!/usr/bin/env python

import rospy
import random
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters
from color_app.msg import CreativeCognitionParameters

class color_shape_detector:

    def __init__(self):
        self.bridge = CvBridge()
        self.success_color = False
        self.success_shape = False
        self.detected_color = ''
        self.detected_shape = ''

        #creating a filter
        #Position 0 is the lower limit and positon 1 the upper one
        blue_threshold = np.array([[103,50,50],[130,255,255]])
        red_threshold = np.array([[169, 100, 100],[189, 255, 255]])
        green_threshold = np.array([[49,50,50],[80, 255, 255]])
        yellow_threshold = np.array([[25,100,100],[35,255,255]])
        #black_threshold = np.array([[0,0,0],[230,25,20]])
        #gray_threshold = np.array([[190,45,35],[220,65,55]])
        #orange_threshold = np.array([[15,65,55],[35,85,75]])
        #pink_threshold = np.array([[290,45,40],[330,85,80]])
        #purple_threshold = np.array([[200,45,40],[240,90,80]])
        self.colors = {'Blue': blue_threshold, 'Red': red_threshold, 'Green': green_threshold, } # 'Orange': orange_threshold, 'Yellow' : yellow_threshold}
        self.shapes = {'Triangle' : 3, 'Square' : 4, 'Circle' : 15}

        #creating a message filter for synchronizing depth an color info
        self.image_sub = rospy.Subscriber("/softkinetic_camera/rgb/image_color", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/softkinetic_camera/depth/image_raw", Image, self.main_callback)

        #creating image kernels for morphological operations
        self.kernel_op = np.ones((2,2),np.uint8)
        self.kernel_cl = np.ones((5,5),np.uint8)

        # image size
        self.image_size = (320,240)

    def image_callback(self, image):
        #convertion from ROS Image format to opencv and filtering
        inImg = self.bridge.imgmsg_to_cv2(image,"bgr8")
        inImg_resized = cv2.resize(inImg, self.image_size, interpolation = cv2.INTER_AREA)

        self.image = inImg_resized

    def main_callback(self, ros_depth):
        ra = rospy.Rate(25) #25hz

        for color in self.colors:

            #depth encoding is "16UC1" rostopic echo /camera/depth/image_raw --noarr shows this encoding
            inDepth = self.bridge.imgmsg_to_cv2(ros_depth, "16UC1")
            inImg_filtered = cv2.GaussianBlur(self.image, (3,3),0)

            #get a Matrix where 0s are the values not included in the range and 1s the included ones
            if inDepth.any() > 0:
                minval =  np.min(inDepth[np.nonzero(inDepth)])
                maxval = minval + 100
                np.place(inDepth, inDepth > maxval, 0)
                np.place(inDepth, inDepth > 0, 1)

            #convertion from rgb to hsv
            inImg_hsv = cv2.cvtColor(inImg_filtered, cv2.COLOR_BGR2HSV)

            #apply the depth mask
            depth_mask_applied = np.multiply(inImg_hsv, inDepth)

            #appliying the color filter
            mask = cv2.inRange(depth_mask_applied,self.colors[color][0], self.colors[color][1])
            cv2.imshow("mask", mask)


            #morphological transformation
            #kernel = np.ones((7,7),np.uint8)
            mask_op = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_op)
            mask_op_cl = cv2.morphologyEx(mask_op, cv2.MORPH_CLOSE, self.kernel_cl)
            #cv2.imshow("mask opening closing", mask_op_cl)


            #removing the small objects from the binary image
            contours,h = cv2.findContours(mask_op_cl,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            mask_final = np.ones(mask_op_cl.shape[:2], dtype="uint8") * 255
            #cv2.imshow("middle step", mask_final)
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
                cv2.drawContours(mask_final, [cnt], -1, 0, -1)
                cv2.bitwise_not(mask_final,mask_final)
                cv2.imshow("hue", mask_final)
                #check if the color filer succeed
                if area_ev > 50:
                    self.detected_color = color
                    self.success_color = True
		elif area_ev < 50 and not self.success_color:
		    self.detected_color = 'None'
		for shape in self.shapes:
		    #appliying the shape filter
		    if(shape == 'Circle') :
		        circles = cv2.HoughCircles(mask_final,cv2.cv.CV_HOUGH_GRADIENT,1,5,param1=20,param2=10,minRadius=5,maxRadius=0)
		        if circles is not None:
		            self.detected_shape = shape
		            self.success_shape = True
		        elif circles is None and not self.success_shape:
		            self.detected_shape = 'None'
		        else :
		            pass
		    if len(cv2.approxPolyDP(cnt,0.1*cv2.arcLength(cnt,True),True)) == self.shapes[shape]:
		        self.detected_shape = shape
		        self.success_shape = True
		    elif not len(cv2.approxPolyDP(cnt,0.1*cv2.arcLength(cnt,True),True)) == self.shapes[shape] and not self.success_shape:
		        self.detected_shape = 'None'
		    else:
		        pass
                self.success_shape = False
                cv2.waitKey(1)
                ra.sleep()
                self.success_color = False

def  color_detection():

    #class definition
    cd = color_shape_detector()

    #ros node defined
    pub = rospy.Publisher('detected_parameters', CreativeCognitionParameters, queue_size = 25)
    rospy.init_node('color_reciever', anonymous = True )
    r = rospy.Rate(25) #25hz
    msg = CreativeCognitionParameters()	
    while not rospy.is_shutdown() :
        print  "Detected " + cd.detected_color + " , " + cd.detected_shape
        msg.Color = cd.detected_color
        msg.Shape = cd.detected_shape
        pub.publish(msg)
        r.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    color_detection()
    rospy.spin()

