#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import dvrk 
import sys

class footpedal:

	def __init__(self, robot_name, ros_namespace = '/'):
		"""Constructor. Initializes data members. It requires just a robot name. ex. r = robot('PSM1')"""
		self.__robot_name = robot_name
		self.__ros_namespace = ros_namespace
		self.__camera_pedal = []
		self.__clutch_pedal = []
		self.__coag_pedal = []
		self.__cam_minus_pedal = []
		self.__headsensor_operator = []
		self.rate = rospy.Rate(2)
		full_ros_namespace = self.__ros_namespace + self.__robot_name

		#subscriber
		rospy.Subscriber(full_ros_namespace + '/camera', Joy, self.current_camera_state_callback, queue_size = 1, buff_size = 1000000)
		rospy.Subscriber(full_ros_namespace + '/clutch', Joy, self.current_clutch_state_callback, queue_size = 1, buff_size = 1000000)
		rospy.Subscriber(full_ros_namespace + '/coag', Joy, self.current_coag_state_callback, queue_size = 1, buff_size = 1000000)
		rospy.Subscriber(full_ros_namespace + '/operatorpresent', Joy, self.current_headsensor_state_callback, queue_size = 1, buff_size = 1000000)
		rospy.Subscriber(full_ros_namespace + '/cam_minus', Joy, self.current_cam_minus_state_callback, queue_size = 1, buff_size = 1000000)
		rospy.Subscriber(full_ros_namespace + '/cam_plus', Joy, self.current_cam_plus_state_callback, queue_size = 1, buff_size = 1000000)

	#callback

	def current_camera_state_callback(self, data):
		self.__camera_pedal = data.buttons

	def current_headsensor_state_callback(self, data):
		self.__headsensor_operator = data.buttons

	def current_clutch_state_callback(self, data):
		self.__clutch_pedal = data.buttons
	
	def current_coag_state_callback(self, data):
		self.__coag_pedal = data.buttons

	def current_cam_minus_state_callback(self, data):
		self.__cam_minus_pedal = data.buttons

	def current_cam_plus_state_callback(self, data):
		self.__cam_plus_pedal = data.buttons

	#getters

	def get_camera_state(self):
		return self.__camera_pedal

	def get_headsensor_state(self):
		return self.__headsensor_operator

	def get_clutch_state(self):
		return self.__clutch_pedal

	def get_coag_state(self):
		return self.__coag_pedal

	def get_cam_plus_state(self):
		return self.__cam_plus_pedal

	def get_cam_minus_state(self):
		return self.__cam_minus_pedal