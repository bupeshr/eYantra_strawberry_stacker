# import the opencv library
import cv2
import cv2.aruco as aruco
from aruco_library import *


# define a video capture object
vid = cv2.VideoCapture("../Test_images/test_vid0.avi")
frame_width = int(vid.get(3))
frame_height = int(vid.get(4))
frame_size = (frame_width,frame_height)
result_vid = "../Test_images/result_vid.avi"

result = cv2.VideoWriter(result_vid, cv2.VideoWriter_fourcc(*'XVID'), 20, frame_size)

while(vid.isOpened() == True):
	
	# Capture the video frame
	# by frame
	ret, frame = vid.read()

	if ret == True:
	# Display the resulting frame
		corners = detect_ArUco(frame)
		angles = Calculate_orientation_in_degree(corners)  
		frame = mark_ArUco(frame, corners, angles)
		result.write(frame)
		cv2.imshow('ID and Angle in deg', frame)


		if cv2.waitKey(30) & 0xFF == ord('q'):
			break
	else:
		break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
