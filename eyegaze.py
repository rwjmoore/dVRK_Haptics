#! /usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray
import time

class Eyegaze:

	def __init__(self, tracker, ros_namespace = '/'):
		"""Constructor. Initializes data membersires just a tracker name. ex. r = robot('gazeTracker')"""
		self.__tracker_name = tracker
		self.__ros_namespace = ros_namespace
		self.__gaze_data = []
		self.rate = rospy.Rate(2)
		full_ros_namespace = self.__ros_namespace + self.__tracker_name

		#subscriber
		rospy.Subscriber(full_ros_namespace, Float64MultiArray, self.current_gaze_state_callback, queue_size = 10, buff_size = 1000000)
		
	#callback

	def current_gaze_state_callback(self, gazedata):
		self.__gaze_data = gazedata.data

	#getters

	def get_gaze_state(self):
		return self.__gaze_data


#Test if subscriber works
# def main():
#     tracker = eyegaze('gazeTracker')
#     while(True):
#         print(tracker.get_gaze_state())
#         time.sleep(1/30)

# if __name__=="__main__":
# 	rospy.init_node('topic_publisher')
# 	rate = rospy.Rate(140)
# 	main()
