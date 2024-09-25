#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import threading
import numpy as np
import dvrk 
import sys
from std_msgs.msg import Bool

class footpedal:

	def __init__(self, ros_namespace = '/footpedals'):
		"""Constructor. Initializes data members. It requires just a robot name. ex. r = robot('PSM1')"""
		self.clutch_pressed = False
		self.operator_present = False
		self.flip = True
		#subscriber
		self.rate = rospy.Rate(100)

		rospy.Subscriber('/footpedals/clutch', Joy, self.clutch_event_cb)
		rospy.Subscriber('/footpedals/operatorpresent', Joy, self.operator_event_cb)
		rospy.Subscriber('/isflipped', Bool, self.flipped_cb)


	def clutch_event_cb(self, data):

		if data.buttons[0] == 1:
			self.clutch_pressed = True
		else:
			self.clutch_pressed = False

	def get_clutch_event(self):

		return self.clutch_pressed 
	
	def flipped_cb(self, data):

		self.flip = data.data

	def get_flip_state(self):

		return self.flip
	
	def operator_event_cb(self, data):

		if data.buttons[0] == 1:
			self.operator_present = True
		else:
			self.operator_present = False

	def get_operator_event(self):

		return self.operator_present 


if __name__ == '__main__':
		
	rospy.init_node('dvrk_mtm_test')
	footpedals = footpedal()
	message_rate = 0.1

	while not rospy.is_shutdown():
		print(footpedals.get_coag_event())