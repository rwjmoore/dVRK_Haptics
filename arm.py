#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import cv2
import numpy as np
import dvrk 
import sys
from scipy.spatial.transform import Rotation as R

class robot:

	def __init__(self, robot_name, ros_namespace = '/'):
		"""Constructor. Initializes data members. It requires just a robot name. ex. r = robot('PSM1')"""
		self.__robot_name = robot_name
		self.__ros_namespace = ros_namespace
		self.__position_joint_current = []
		self.__measured_joint = []
		self.__position_jaw_current = []
		self.__position_cartesian_current = []
		self.__orientation_current = []
		self.seq = 0
		self.rate = rospy.Rate(2)
		self.count = -1
		self.nsecs = 0
		self.secs = 0
		self.start_time = 0
		self.full_array = []
		self.jaw_name = []
		self.joint_name = []
		full_ros_namespace = self.__ros_namespace + self.__robot_name

		#subscriber
		rospy.Subscriber(full_ros_namespace + '/measured_js', JointState, self.current_joint_state_callback, queue_size = 1, buff_size = 1000000)
		rospy.Subscriber(full_ros_namespace + '/jaw/measured_js', JointState, self.current_jaw_state_callback, queue_size = 1, buff_size = 1000000)
		rospy.Subscriber(full_ros_namespace + '/measured_cp', TransformStamped, self.current_cartesian_position_callback, queue_size = 1, buff_size = 1000000)
		#callback

	def current_joint_state_callback(self, data):
		self.__position_joint_current[:] = data.position
		self.full_array.append(self.__position_joint_current[:])
		self.seq = data.header.seq
		self.nsecs = data.header.stamp.nsecs
		self.secs = data.header.stamp.secs
		self.start_time = 0
		self.joint_name = data.name

	def current_cartesian_position_callback(self, data):

		self.__position_cartesian_current = [data.transform.translation.x, data.transform.translation.y, data.transform.translation.z]
		self.__orientation_current = [data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
		
	def current_jaw_state_callback(self, data):

		self.__position_jaw_current[:] = data.position
		self.jaw_name = data.name

	#getters
	def get_jaw_name(self):
		return self.jaw_name

	def get_joint_name(self):
		return self.joint_name
		
	def get_current_cartesian_position(self):

		return self.__position_cartesian_current

	def get_current_orientation_matrix(self):

		r = R.from_quat(self.__orientation_current)
		rot_matrix = r.as_matrix().flatten()
		return rot_matrix

	def get_current_joint_position(self):
		
		return self.__position_joint_current

	def get_current_jaw_position(self):
			
		return self.__position_jaw_current
		
	def get_seq(self):
		
		return self.seq

	def increment_count(self):
		
		self.count = self.count + 1
		
		return self.count

	def get_nsecs(self):
		
		return self.nsecs*10**(-9)

	def get_secs(self):
		
		return self.secs

	def get_time(self):

		self.time = self.get_secs() + self.get_nsecs()
		return self.time

	def get_start_time(self):
		
		if self.count == 1:
			self.start_time = self.get_time()

		return self.start_time

	def get_count(self):
		return self.count

	def get_full_array(self):
		return self.full_array
