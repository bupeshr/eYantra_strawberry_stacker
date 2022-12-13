#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
'''
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from aruco_library import *


class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('marker_detection') #Initialise rosnode 
		
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
        	# Subscribing to /camera/camera/image_raw

		self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
	        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		
		self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker


	# Callback function of camera topic
	def image_callback(self, data):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except CvBridgeError as e:
			print(e)
			return

		ard = detect_ArUco(self.img)
		arg = Calculate_orientation_in_degree(ard)
		id = list(ard.keys())[0]
		corners = list(ard.values())[0]
		angle = list(arg.values())[0]
		# print(arg)

		#from aruco_library
		top_left, top_right, bottom_right, bottom_left = corners

		top_left = (int(top_left[0]),int(top_left[1]))
		top_right = (int(top_right[0]), int(top_right[1])) 
		bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
		bottom_left = (int(bottom_left[0]), int(bottom_left[1]))

		#center(x,y) of the marker
		center_x = int((top_left[0]+bottom_right[0])/2)
		center_y = int((top_left[1]+bottom_right[1])/2)

		#for angle
		if angle < 0:
			angle = 360 - abs(angle)
		else:
			angle = angle


		#publishing

		self.marker_msg.id = id
		self.marker_msg.x = center_x
		self.marker_msg.y = center_y
		self.marker_msg.yaw = angle
		print(self.marker_msg)
		print("-----------------")
		self.publish_data(self.marker_msg)
		# print(corners, id, angle)
			
	def publish_data(self, marker_msg):
		self.marker_pub.publish(self.marker_msg)

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
