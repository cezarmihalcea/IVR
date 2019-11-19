import image1
import image2

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class angle_calculator
	
	def __init__(self):
		self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
		self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
		self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback2)
		self.bridge = CvBridge()

	def detect3dyellow(self, img1, img2):
		yellowXY = image1.detect_yellow(img1)
		yellowZY = image2.detect_yellow(img2)

		xyz = np.array([yellowXY[0], yellowXY[1], yellowZY[0]])

	def detect3dred(self, img1, img2):
		redXY = image1.detect_red(img1)
		redZY = image2.detect_red(img2)

		xyz = np.array([redXY[0], redXY[1], redZY[0]])

	def detect3dblue(self, img1, img2):
		blueXY = image1.detect_blue(img1)
		blueZY = image2.detect_blue(img2)

		xyz = np.array([blueXY[0], blueXY[1], blueZY[0]])

	def detect3dgreen(self, img1, img2):
		greenXY = image1.detect_green(img1)
		greenZY = image2.detect_green(img2)

		xyz = np.array([greenXY[0], greenXY[1], greenZY[0]])

	def unitvector(vector):
		return vector / np.linalg.norm(vector)

	def anglebetween(v1, v2):
		uv1 = unitvector(v1)
		uv2 = unitvector(v2)
		return np.arccos(np.clip(np.dot(uv1, uv2), -1.0, 1.0))

	def jointangles(self, img1, img2):
		yellow = self.detect3dyellow(img1, img2)
		red = self.detect3dred(img1, img2)
		blue = self.detect3dblue(img1, img2)
		green = self.detect3dgreen(img1, img2)

		vectYB = blue - yellow
		vectBG = green - blue
		vectGR = red - green

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

		angle1 = np.arccos(0)
		angle2 = anglebetween(vectYB, vectBG)
		angle3 = anglebetween(vectBG, vectGR)

	def callback(self, data):
		try:
	      cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
	      cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
	    except CvBridgeError as e:
	      print(e)