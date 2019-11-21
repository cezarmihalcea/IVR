#!/usr/bin/env python

import image1
import image2

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class angle_calculator:
	
	def __init__(self):
		rospy.init_node('image_processing', anonymous=True)

		self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)

		self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw", Image)
		self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw", Image)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2], 10, 1, allow_headerless=True)
		self.ts.registerCallback(self.callback)

		self.bridge = CvBridge()
		self.time_trajectory = rospy.get_time()

	def detectOrange(self, img):
		mask = cv2.inRange(image, (0,100,200), (0,180,240))
  
		kernel = np.ones((5, 5), np.uint8)
		mask = cv2.dilate(mask, kernel, iterations = 3)

		m = cv2.moments(mask)

		cz = int(m['m10'] / m['m00'])
		cy = int(m['m01'] / m['m00'])

		return np.array([cx, cy])

	def greater(a, b):
	    momA = cv2.moments(a)        
	    (xa,ya) = int(momA['m10']/momA['m00']), int(momA['m01']/momA['m00'])

	    momB = cv2.moments(b)        
	    (xb,yb) = int(momB['m10']/momB['m00']), int(momB['m01']/momB['m00'])

	    if xa > xb:
	        return 1

	    if xa == xb:
	        return 0
	    else:
	        return -1

	def squarecontours(self, img, tmp):
		center = self.detectOrange(img)

		mask = cv2.inRange(img, (0, 0, 0), (1, 1, 1))
		contours = cv2.findContours(mask, mode=CV_RETR_LIST, method=CV_CHAIN_APPROX_SIMPLE)

		cntsSorted = contours.sort(greater)
		sqcnt = cntsSorted[1]

	def detect3dyellow(self, img1, img2):
		yellowXY = image1.detect_yellow(img1)
		yellowZY = image2.detect_yellow(img2)

		xyz = np.array([yellowXY[0], yellowXY[1], yellowZY[0]])
		return xyz

	def detect3dred(self, img1, img2):
		redXY = image1.detect_red(img1)
		redZY = image2.detect_red(img2)

		xyz = np.array([redXY[0], redXY[1], redZY[0]])
		return xyz

	def detect3dblue(self, img1, img2):
		blueXY = image1.detect_blue(img1)
		blueZY = image2.detect_blue(img2)

		xyz = np.array([blueXY[0], blueXY[1], blueZY[0]])
		return xyz

	def detect3dgreen(self, img1, img2):
		greenXY = image1.detect_green(img1)
		greenZY = image2.detect_green(img2)

		xyz = np.array([greenXY[0], greenXY[1], greenZY[0]])
		return xyz

	def unitvector(self, vector):
		return vector / np.linalg.norm(vector)

	def anglebetween(self, v1, v2):
		uv1 = self.unitvector(v1)
		uv2 = self.unitvector(v2)
		return np.arccos(np.clip(np.dot(uv1, uv2), -1.0, 1.0))

	def jointangles(self, img1, img2):
		yellow = self.detect3dyellow(img1, img2)
		red = self.detect3dred(img1, img2)
		blue = self.detect3dblue(img1, img2)
		green = self.detect3dgreen(img1, img2)

		bluecm1 = np.array([blue[0], blue[1], 0])
		bluecm2 = np.array([0, blue[1], blue[2]])

		greencm1 = np.array([green[0], green[1], 0])
		greencm2 = np.array([0, green[1], green[2]])

		vectBG1 = bluecm1 - greencm1
		vectBG2 = bluecm2 - greencm2

		vectYB = blue - yellow
		vectBG = green - blue
		vectGR = red - green

		vect0 = np.array([vectBG[0], 0, vectBG[2]])
		vectQ = np.array([1, 0, 0])

		angle1 = self.anglebetween(vect0, vectQ)
		angle2 = self.anglebetween(vectYB, vectBG1) - angle1
		angle3 = self.anglebetween(vectYB, vectBG2) - angle1
		angle4 = self.anglebetween(vectBG, vectGR) - angle1 - angle2

		return np.array([angle1, angle2, angle3, angle4])

	def callback(self, data1, data2):
		try:
			self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
			self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
		except CvBridgeError as e:
			print(e)

		#self.link1 = cv2.inRange(cv2.imread('link1.png', 1), (200, 200, 200), (255, 255, 255))
		#self.link2 = cv2.inRange(cv2.imread('link2.png', 1), (200, 200, 200), (255, 255, 255))

		jointsData = self.jointangles(self.cv_image1, self.cv_image2)

		self.joints = Float64MultiArray()
		self.joints.data = jointsData

		try:
			self.joints_pub.publish(self.joints)
		except CvBridgeError as e:
			print(e)

def main(args):
	  ic = angle_calculator()
	  try:
		rospy.spin()
	  except KeyboardInterrupt:
		print("Shutting down")
	  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
