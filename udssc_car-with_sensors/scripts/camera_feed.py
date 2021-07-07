#!/usr/bin/env python

import cv2
import rospy
import platform
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

class camera_feed(object):

	def __init__(self):

		# ROS Node
		hostname =platform.node()
		node_name = hostname + "_camera_feed"
		rospy.init_node(node_name, anonymous=False, log_level=rospy.DEBUG)
		self.rate = rospy.Rate(30)

		# Videofeed
		self.vid = cv2.VideoCapture(0)
		self.bridge = CvBridge()

		# ROS Topic
		self.pub_image = rospy.Publisher("camera_feed", Image, queue_size=1)

	def loop(self):
		""" Main Camera Loop """

		ret, frame = self.vid.read()

		frame = cv2.resize(frame,(200,200))

		msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
		self.pub_image.publish(msg)

if __name__ == "__main__":
	feed = camera_feed()
	while not rospy.is_shutdown():
		feed.loop()
	cap.release()
