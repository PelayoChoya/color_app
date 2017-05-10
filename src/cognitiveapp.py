#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import String
from color_app.msg import CreativeCognitionParameters

class cognitive_helper:

	def __init__(self):

		self.colors = ('Blue', 'Red', 'Green')
		self.shapes = ('Triangle', 'Square', 'Circle')
		self.goal_color = ''
		self.goal_shape = ''
		self.success_color = False
		self.success_shape = False
		self.topic_sub = rospy.Subscriber('detected_parameters', CreativeCognitionParameters, self.callback)
	
	def random_election(self):

		self.goal_color = random.choice(self.colors)
		self.goal_shape = random.choice(self.shapes)

	def reset_parameters(self):
		self.success_color = False
		self.success_shape = False

	def callback(self, data):
		if data.Color == self.goal_color:
			self.success_color = True
		else:
			self.success_color = False
		if data.Shape == self.goal_shape:
			self.success_shape = True
		else:
			self.success_shape = False
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s, %s" , data.Color, data.Shape)
 	
def cognitive_process():
	ch = cognitive_helper()
	#ROS parameters definition
	rospy.init_node('cognitive_application', anonymous = True)
	r = rospy.Rate(10) #10hz
	while not rospy.is_shutdown() :
		ch.random_election()
		print "Show me the " + ch.goal_color + "color, " + ch.goal_shape+ " shape" 
		while (ch.success_color == False and ch.success_shape == False and not rospy.is_shutdown()):
			print "Wrong"
			print "Requested " + ch.goal_color + "color, " + ch.goal_shape+ " shape" 	
			r.sleep()

if __name__ == '__main__':
		cognitive_process()
		rospy.spin()
	
