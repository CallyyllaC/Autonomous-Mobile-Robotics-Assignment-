#!/usr/bin/env python

#GitHub. 2020. LCAS/Teaching. [online] Available at: <https://github.com/LCAS/teaching> [Accessed 14 March 2020].

import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # enables for bridging to openCV
import numpy as np


class opencvBridge():

	def __init__(self):
	# __init__ is a constructor in object orientated concepts
	# self is an instance of the class (opencv_bridge)... 
	# ...We can access attributes and methods of the class

		self.node_name = "opencv_bridge" # given name for node. Will appear as this in rqt_graph
		rospy.init_node(self.node_name) # initialise the node

		self.cv_window_name = self.node_name # Create the OpenCV display window for the RGB image
		self.bridge = CvBridge() # Create the cv_bridge object

		rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback) # sub to topic
		rospy.Timer(rospy.Duration(0.03), self.open_windows) # timer for displaying windows

	def open_windows(self,event): # define the function 'open_windows'
		try:

			# WINDOW_NORMAL = allows for the window to be resized
			cv2.namedWindow("Cammy", cv2.WINDOW_NORMAL)
			cv2.namedWindow("Canny", cv2.WINDOW_NORMAL)

			cv2.imshow("Cammy",self.cam_view)
			cv2.imshow("Canny",self.processed_image)

			cv2.waitKey(3)

		except:
			pass

	def image_callback(self, data): # define the function 'image_callback'
		# Use cv_bridge() to convert the ROS image to OpenCV format
		try:
			self.cam_view = self.bridge.imgmsg_to_cv2(data, "bgr8") # convert to RGB 8-bit data type 
		except CvBridgeError, e: # if we cannot receive a data input print e
			print e
			pass

		cam_view = np.array(self.cam_view, dtype = np.uint8)
		self.processed_image = self.edge_detection(cam_view)

	def edge_detection(self, cam_view): # define the function 'edge detection'
		gray_img = cv2.cvtColor(cam_view, cv2.COLOR_BGR2GRAY)
		gray_img = cv2.blur(gray_img, (3, 3)) # helps to reduce noise w/ 3*3 gaussian kernel
		edges = cv2.Canny(gray_img, 10, 200) # set gradients

		return edges

opencvBridge() # callback to class ^^
rospy.spin() # keeps the node from exiting until the node is shutdown

# (Quigley et al., 2015, 193-208)
# (Lentin, 2018, 176-189)

"""
Quigley, M., Gerkey, B. and Smart, W.D. (2015) Programming Robots with ROS.
Sebastopol, USA: O'Reilly Media, Inc.

Lentin, J. (2018) Learning Robotics using Python, 2nd Edition.
Birmingham, UK: Packt Publishing Ltd.

"""
