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
		self.square_pub = rospy.Publisher("square_pos", Float64MultiArray, queue_size=12)

		self.image_sub1 = message_filters.Subscriber("/camera1/robot/image_raw", Image)
		self.image_sub2 = message_filters.Subscriber("/camera2/robot/image_raw", Image)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2], 10, 1, allow_headerless=True)
		self.ts.registerCallback(self.callback)

		self.bridge = CvBridge()
		self.time_trajectory = rospy.get_time()

	def detectOrange(self, img):
		mask = cv2.inRange(img, (10,100,160), (100,200,255))
  
		kernel = np.ones((5, 5), np.uint8)
		mask = cv2.dilate(mask, kernel, iterations = 3)

		#m = cv2.moments(mask)

		#cx = int(m['m10'] / m['m00'])
		#cy = int(m['m01'] / m['m00'])

		return mask

	def squarecontours(self, img):
		center = self.detectOrange(img)
		contours, hierarchy = cv2.findContours(center, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

		sqcnt = None
		for contour in contours:
			approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True), True)
			if len(approx) >= 8:
				sqcnt = contour

		if sqcnt is not None:
			m = cv2.moments(sqcnt)

			cx = int(m['m10'] / m['m00'])
			cy = int(m['m01'] / m['m00'])

			return np.array([cx, cy])
		else:
			return np.array([0, 0])

	def squarepos(self, img1, img2):
		pos1 = self.squarecontours(img1)
		pos2 = self.squarecontours(img2)

		return np.array([pos1[0], pos2[0], max(pos1[1], pos2[1])])

	def dist_sq_robot(self, img1, img2):
		pos = self.squarepos(img1, img2)
		yellowpos = self.detect3dyellow(img1, img2)
		dist = np.sum((pos - yellowpos)**2)
		return np.array([float(pos[0]) / 100, float(pos[1]) / 100, float(pos[2]) / 100, np.sqrt(dist) / 100])


	def detect3dyellow(self, img1, img2):
		yellowXY = image1.detect_yellow(img1)
		yellowZY = image2.detect_yellow(img2)

		xyz = np.array([yellowXY[0], max(yellowXY[1], yellowZY[1]), yellowZY[0]])
		return xyz

	def detect3dred(self, img1, img2):
		redXY = image1.detect_red(img1)
		redZY = image2.detect_red(img2)

		xyz = np.array([redXY[0], max(redXY[1], redZY[1]), redZY[0]])
		return xyz

	def detect3dblue(self, img1, img2):
		blueXY = image1.detect_blue(img1)
		blueZY = image2.detect_blue(img2)

		xyz = np.array([blueXY[0], max(blueXY[1], blueZY[1]), blueZY[0]])
		return xyz

	def detect3dgreen(self, img1, img2):
		greenXY = image1.detect_green(img1)
		greenZY = image2.detect_green(img2)

		xyz = np.array([greenXY[0], max(greenXY[1], greenZY[1]), greenZY[0]])
		return xyz

	def unitvector(self, vector):
		return vector / np.linalg.norm(vector)

	def dotproduct(self, v1, v2):
		return np.sum(v1*v2)

	def length(self, v):
		return np.sqrt(self.dotproduct(v, v))

	def anglebetween(self, v1, v2):
		return np.arccos(self.dotproduct(v1, v2) / (self.length(v1) * self.length(v2)))

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

		angle3 = self.anglebetween(vectBG, np.array([vectBG[0], vectBG[1], 0]))
		angle2 = self.anglebetween(vectYB, np.array([vectBG[0], vectBG[1], 0]))

		angle4 = self.anglebetween(vectBG, vectGR)
		#angle2 = self.anglebetween(vectYB, vectBG1)

		angle1 = (3.14 - self.anglebetween(vect0, vectQ)) - angle3
		return np.array([angle1, angle2, angle3, angle4])

	def callback(self, data1, data2):
		try:
			self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
			self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
		except CvBridgeError as e:
			print(e)

		jointsData = self.jointangles(self.cv_image1, self.cv_image2)

		self.joints = Float64MultiArray()
		self.joints.data = jointsData

		print(self.joints)

		self.squaredistance = Float64MultiArray()
		self.squaredistance.data = self.dist_sq_robot(self.cv_image1, self.cv_image2)

		try:
			self.joints_pub.publish(self.joints)
			self.square_pub.publish(self.squaredistance)
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
