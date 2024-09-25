#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import cv2
import numpy as np
import xlsxwriter
import dvrk 
import sys
from scipy.spatial.transform import Rotation as R
import os

class camera:


	def __init__(self, camera_name, ros_namespace = '/dVRK/'):

		self.__camera_name = camera_name
		self.__ros_namespace = ros_namespace
		self.bridge = CvBridge()
		self.cv_image = []
		self.image_count = 1
		self.image_path = os.path.abspath(os.getcwd()) + '/Images/'


		full_ros_namespace = self.__ros_namespace + self.__camera_name + '/decklink/camera'

		#subscriber
		rospy.Subscriber(full_ros_namespace+ '/image_raw', Image, self.image_callback, queue_size = 1, buff_size = 1000000)

	def image_callback(self, data):

		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

	#saves the image in a folder.
#/dVRK/left/decklink/camera/image_raw
		#getters
	def get_image(self):
		return self.cv_image

	def save_image(self):

		if self.cv_image.size != 0:
			cv2.imwrite(self.image_path + self.__camera_name+"/"+self.__camera_name+"_Camera" +"_" + str(self.image_count)+".png", self.cv_image)
			self.image_count = self.image_count + 1
			return True
		else:
			return False