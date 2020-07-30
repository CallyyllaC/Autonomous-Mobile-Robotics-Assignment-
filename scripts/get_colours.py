import rospy
import sys
import cv2
import numpy as np
from numpy import mean
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#class used for image processing
#this class was based off our solution of the in class task 3
#GitHub. 2020. Callyyllac/Cmp3103_Amr_19-20. [online] Available at: <https://github.com/CallyyllaC/cmp3103_amr_19-20> [Accessed 14 March 2020].
class eyes():

	def __init__(self):

		self.cv_window_name = "Eyes"
		self.bridge = CvBridge()

		#output is published as a float
		#output avg colour
		self.pub_red = rospy.Publisher("/result_red", Float32, queue_size=1)
		self.pub_blue = rospy.Publisher("/result_blue", Float32, queue_size=1)
		self.pub_green = rospy.Publisher("/result_green", Float32, queue_size=1)
		#output is published as twist
		#output corrective angle
		self.pub_toblue = rospy.Publisher("/to_blue", Twist, queue_size=1)
		self.pub_togreen = rospy.Publisher("/to_green", Twist, queue_size=1)
		
		#subscribe to the camera
		self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
		rospy.Timer(rospy.Duration(0.03), self.open_windows) # timer for displaying windows

	#display the images
	def open_windows(self,event):
		try:
            #create a window with the default robot view
			cv2.namedWindow("Raw", cv2.WINDOW_NORMAL)
			#create a window with the altered robot view
			cv2.namedWindow("Floor", cv2.WINDOW_NORMAL)
			#create a window with the altered robot view
			cv2.namedWindow("Ahead", cv2.WINDOW_NORMAL)
			#create a window with the altered robot view
			cv2.namedWindow("Goal", cv2.WINDOW_NORMAL)
			#show the images
			cv2.imshow("Raw",self.cam_view)					#raw image
			cv2.imshow("Floor",self.processed_image_floor)	#image of traps
			cv2.imshow("Ahead",self.processed_image_ahead)	#image of clues
			cv2.imshow("Goal",self.processed_image_goal)	#image of finish
			cv2.waitKey(3)
        #catch errors
		except:
			pass

	#image callback
	def image_callback(self, data):
		#convert from ros image to open cv image type and store
		try:
			self.cam_view = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e
			pass

		#transforms the image into an 8b numpy array
		cam_view = np.array(self.cam_view, dtype = np.uint8)
		#get the processed images
		self.processed_image_ahead, self.processed_image_floor, self.processed_image_goal = self.color(cam_view)

	def color(self, cam_view):
		#convert from bgr to hsv
		hsv = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)
	
		#get the range of each colour we are looking for in hsv
        #the colours used were found at:
		#Rapidtables.com. 2020. RGB To HSV Conversion | Color Conversion. [online] Available at: <https://www.rapidtables.com/convert/color/rgb-to-hsv.html> [Accessed 14 March 2020].
		lower_blue = np.array([110, 50, 0])
		upper_blue = np.array([130, 255, 255])
		lower_red = np.array([0, 50, 0])
		upper_red = np.array([10, 255, 255])
		lower_green = np.array([45, 50, 0])
		upper_green = np.array([65, 255, 255])
		
		#turn ranges into a mask
		blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
		red_mask = cv2.inRange(hsv, lower_red, upper_red)
		green_mask = cv2.inRange(hsv, lower_green, upper_green)
		
		#get the height, width and depth of the image (only actually need width)
		h, w, d = cam_view.shape

		#create twist holders
		t_green = Twist()
		t_blue = Twist()
		
		#get the correction and draw the detection on the image
		self.get_track(red_mask, w, "Trap")
		#only get the returns of green and blue as we want to move away from red
		t_green.angular.z = self.get_track(green_mask, w, "Finish")/4
		t_blue.angular.z = self.get_track(blue_mask, w, "Hint")/4
		
		#add masks to images
		blue_masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask=blue_mask)
		red_masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask=red_mask)[300:480,:]
		green_masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask=green_mask)[300:480,:]
		
		#publish the means
		self.pub_blue.publish(np.mean(blue_masked[:, :, 0]))
		self.pub_green.publish(np.mean(green_masked[:, :, 0]))
		self.pub_red.publish(np.mean(red_masked[:, :, 0]))

		#if there is a correction to blue publish
		if(t_blue.angular.z != 0):
			self.pub_toblue.publish(t_blue)

		#if there is a correction to green publish
		if(t_green.angular.z != 0):
			self.pub_togreen.publish(t_green)
		
		#return the masked images for display
		return blue_masked, red_masked, green_masked

	#Function used to find the center of the found colour and get the rotation towards it
    #this function is based off of code from:
	#Consulting, A., 2020. Find Center Of A Blob (Centroid) Using Opencv (C++/Python) | Learn Opencv. [online] Learnopencv.com. Available at: <https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/> [Accessed 16 March 2020].
	def get_track(self, m, w, text):
		#get moments of mask
		M = cv2.moments(m)
		#if needs adjustment
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00']) #y isnt needed for the robot, only for the visualisation
			#draw the circle in the middle
			cv2.circle(self.cam_view, (cx, cy), 5, (255, 255, 255), -1)
			#add the label
			cv2.putText(self.cam_view, text,(cx, cy), cv2.FONT_HERSHEY_PLAIN, 5, (255, 255, 255), 2)
			#work out the error and how to move towards it (correct rotation)
			err = cx - w/2
			return -float(err) / 100
		else:
			return 0