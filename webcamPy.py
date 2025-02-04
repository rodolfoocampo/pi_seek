# import the necessary packages
from collections import deque
import numpy as np
import argparse
#import imutils
import cv2

def findCenter(): 
	# construct the argument parse and parse the arguments
	#ap=argparse.ArgumentParser()
	#ap.add_argument("-v", "--video",help="path to the (optional) video file")
	#args = vars(ap.parse_args())


	# define the lower and upper boundaries of the "green"
	# ball in the HSV color space, then initialize the
	# list of tracked points
	greenLower = (147, 121, 0)
	greenUpper = (256, 256, 256)

	 
	# if a video path was not supplied, grab the reference
	# to the webcam
	#if not args.get("video", False):
	camera = cv2.VideoCapture(0)
	 
	# otherwise, grab a reference to the video file
	#else:
	#camera = cv2.VideoCapture(args["video"])

	# keep looping
	while True:
		puntos=[]
		puntos.append(0)
		puntos.append(0)	
		# grab the current frame
		(grabbed, frame) = camera.read(0)
		
	 
		# if we are viewing a video and we did not grab a frame,
		# then we have reached the end of the video
		#if args.get("video") and not grabbed:
			#break
	 
		# resize the frame, blur it, and convert it to the HSV
		# color space
#		frame = imutils.resize(frame, width=640)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	 
		# construct a mask for the color "green", then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, greenLower, greenUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
	
		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None
	 
		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	 
			# only proceed if the radius meets a minimum size
			if radius > 50 and radius < 350 and x!=0 and y!=0:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				puntos[0]=(320-x)*(0.004777)
				puntos[1]=(240-y)*(0.004328)			
				#cv2.circle(frame, (int(x), int(y)), int	(radius),(0, 255, 255), 2)
				#cv2.circle(frame, center, 5, (0, 0, 255), -1)
	 			return puntos
				break
		# show the frame to our screen
		#cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF
		# if the 'q' key is pressed, stop the loop
		if key == ord("q"):
			break
	 
	# cleanup the camera and close any open windows
	camera.release()
	cv2.destroyAllWindows()
