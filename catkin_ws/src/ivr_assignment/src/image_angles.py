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

		vectYB = blue - yellow
		vectBG = green - blue
		vectGR = red - green

		vect0 = np.array([vectYB[0], 0, vectYB[2]])

		'''
		distYB = 2 / np.sqrt(np.sum(vectYB**2))
		distBG = 3 / np.sqrt(np.sum(vectBG**2))
		distGR = 2 / np.sqrt(np.sum(vectGR**2))
		dist0 = 2 / np.sqrt(np.sum(vect0**2))

		dotYBG = np.dot(vectYB, vectBG)
		dotBGR = np.dot(vectBG, vectGR)
		dot0YB = np.dot(vect0, vectYB)

		cos0YB = dot0YB / (dist0 * distYB)
		cosYBG = dotYBG / (distYB * distBG)
		cosBGR = dotBGR / (distBG * distGR)
		'''

		angle1 = self.anglebetween(vect0, vectYB)
		angle2 = self.anglebetween(vectYB, vectBG) - angle1
		angle3 = self.anglebetween(vectBG, vectGR) - angle1 - angle2

		return np.array([angle1, angle2, angle3])

	def callback(self, data1, data2):
		try:
			self.cv_image1 = self.bridge.imgmsg_to_cv2(data1, "bgr8")
			self.cv_image2 = self.bridge.imgmsg_to_cv2(data2, "bgr8")
		except CvBridgeError as e:
			print(e)

		#im1 = cv2.imshow('window1', self.cv_image1)
		#im2 = cv2.imshow('window2', self.cv_image2)

		jointsData = self.jointangles(self.cv_image1, self.cv_image2)

		self.joints = Float64MultiArray()
		self.joints.data = jointsData

		try:
			#self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
			#self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
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
