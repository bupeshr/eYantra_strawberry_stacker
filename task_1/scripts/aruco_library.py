#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
	## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163],
	#							[319, 263],
	#							[219, 267],
	#							[215,167]], dtype=float32)}
	Detected_ArUco_markers = {}

	## enter your code here ##
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	aruco_det = aruco.Dictionary_get(aruco.DICT_5X5_250)
	aruco_param = aruco.DetectorParameters_create()
	corners, ids, _ = aruco.detectMarkers(gray, aruco_det, parameters =aruco_param)
	# aruco.drawDetectedMarkers(img, corners, ids)
	# print("before: ",ids)

	if np.any(ids!= None):
		ids = ids.flatten()
		for (corners1, ids1) in zip(corners, ids):
			corners1 = corners1.reshape((4, 2))
			Detected_ArUco_markers[ids1] = corners1
	# print("after: ",ids1)
	# print(Detected_ArUco_markers)
	return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}

	ArUco_marker_angles = {}
	## enter your code here ##
	if len(Detected_ArUco_markers)>0:
		for ids, corners in Detected_ArUco_markers.items():
			if ids != None:
				top_left, top_right, bottom_right, bottom_left = corners

				top_left = (int(top_left[0]),int(top_left[1]))
				top_right = (int(top_right[0]), int(top_right[1])) 
				bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
				bottom_left = (int(bottom_left[0]), int(bottom_left[1]))


				#center(x,y) of the marker
				center_x = int((top_left[0]+bottom_right[0])/2)
				center_y = int((top_left[1]+bottom_right[1])/2)

				#mid-point in the top of the marker between top left and top right
				mid_x = int((top_left[0]+top_right[0])/2)
				mid_y = int((top_left[1]+top_right[1])/2)

				#angle of rotation with respect to the center and the mid-point
				angle_in_rad = math.atan2((center_y-mid_y), (mid_x-center_x))

				#angle in degree
				angle_in_deg = int(angle_in_rad*(180/math.pi))

				if (mid_x-center_x) < 0 and (center_y-mid_y) >= 0:
					angle_in_deg = angle_in_deg
				elif (mid_x - center_x) < 0 and (center_y - mid_y) <0:
					angle_in_deg = 360 + angle_in_deg
				elif (mid_x - center_x) > 0 and (center_y - mid_y) <0:
					angle_in_deg = 360 + angle_in_deg
				


				ArUco_marker_angles[ids]= int(angle_in_deg)
		# print(ArUco_marker_angles)


	return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement

    ## enter your code here ##
	if len(Detected_ArUco_markers)>0:
		for ids, corners in Detected_ArUco_markers.items():
		
			top_left, top_right, bottom_right, bottom_left = corners

			top_left = (int(top_left[0]),int(top_left[1]))
			top_right = (int(top_right[0]), int(top_right[1])) 
			bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
			bottom_left = (int(bottom_left[0]), int(bottom_left[1]))

			#center(x,y) of the marker
			center_x = int((top_left[0] + bottom_right[0])/2)
			center_y = int((top_left[1] + bottom_right[1])/2)
			# print("center: ", type(center_x), type(center_y))
			center= (center_x,center_y)

			#mid-point in the top of the marker
			mid_x = int((top_left[0] + int(top_right[0]))/2)
			mid_y = int((top_left[1] + int(top_right[1]))/2)
			mid_point = (mid_x,mid_y)

			cv2.circle(img, top_left, 5, (90,90,90),-1) #Gray
			cv2.circle(img, top_right, 5, (0,255, 0), -1) #Green
			cv2.circle(img, bottom_right, 5, (255,153,255), -1) #Pink
			cv2.circle(img, bottom_left, 5, (255,255,255), -1) #White
			cv2.circle(img, center, 2,(0, 0, 255),2) #red circle in the center

			cv2.line(img, center, mid_point, (255,0,0), 2) #Blue line
			
			cv2.putText(img, str(ids), (center_x+15, center_y), cv2.FONT_HERSHEY_PLAIN, 2,(0,0,235),4)
			cv2.putText(img, str(ArUco_marker_angles[ids]), (center_x-50, center_y+10), cv2.FONT_HERSHEY_PLAIN, 1.5, (0,255, 0), 2)
			# cv2.circle(img, mid_point, 2, (255,0,0), 2)
				
		

	return img


