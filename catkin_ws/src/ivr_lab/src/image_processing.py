#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send messages to a topic named image_topic
    self.image_pub = rospy.Publisher("image_topic",Image, queue_size = 1)
    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    # initialize a publisher to send robot end-effector position
    self.end_effector_pub = rospy.Publisher("end_effector_prediction",Float64MultiArray, queue_size=10)
    # initialize a publisher to send desired trajectory
    self.trajectory_pub = rospy.Publisher("trajectory",Float64MultiArray, queue_size=10)
    # initialize a publisher to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub = rospy.Subscriber("/robot/camera1/image_raw",Image,self.callback)
    # record the begining time
    self.time_trajectory = rospy.get_time()
  
  # Define a circular trajectory (for lab 3)
  def trajectory(self):
    # get current time
    cur_time = np.array([rospy.get_time() - self.time_trajectory])
    x_d = float(6* np.cos(cur_time * np.pi/100))
    y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
    return np.array([x_d, y_d])
  
  def detect_green(self, image):
    mask = cv2.inRange(image, (0,100,0), (0,255,0))
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations = 3)
    
    m = cv2.moments(mask)
    
    cx = int(m['m10'] / m['m00'])
    cy = int(m['m01'] / m['m00'])
    
    return np.array([cx, cy])
  
  def detect_red(self, image):
    mask = cv2.inRange(image, (0,0,100), (0,0,255))
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations = 3)
    
    m = cv2.moments(mask)
    
    cx = int(m['m10'] / m['m00'])
    cy = int(m['m01'] / m['m00'])
    
    return np.array([cx, cy])
  
  def detect_blue(self, image):
    mask = cv2.inRange(image, (100,0,0), (255,0,0))
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations = 3)
    
    m = cv2.moments(mask)
    
    cx = int(m['m10'] / m['m00'])
    cy = int(m['m01'] / m['m00'])
    
    return np.array([cx, cy])
  
  def detect_yellow(self, image):
    mask = cv2.inRange(image, (0,100,100), (0,255,255))
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations = 3)
    
    m = cv2.moments(mask)
    
    cx = int(m['m10'] / m['m00'])
    cy = int(m['m01'] / m['m00'])
    
    return np.array([cx, cy])
  
  def pixel2meter(self, image):
	circle1pos = self.detect_blue(image)
	circle2pos = self.detect_green(image)
	
	dist = np.sum((circle1pos - circle2pos)**2)
	
	return 3 / np.sqrt(dist)
	
  def detect_joint_angles(self, image):
    a = self.pixel2meter(image)
	
    center = a * self.detect_yellow(image)
    circle1pos = a * self.detect_blue(image)
    circle2pos = a * self.detect_green(image)
    circle3pos = a * self.detect_red(image)
    
    ja1 = np.arctan2(center[0] - circle1pos[0], center[1] - circle1pos[1])
    ja2 = np.arctan2(circle1pos[0] - circle2pos[0], circle1pos[1] - circle2pos[1]) - ja1
    ja3 = np.arctan2(circle2pos[0] - circle3pos[0], circle2pos[1] - circle3pos[1]) - ja1 - ja2
    
    return np.array([ja1, ja2, ja3])
  
  # Recieve data, process it, and publish
  def callback(self,data):
    # Recieve the image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
      
    # loading template for links as binary image (used in lab 2)
    # self.link1 = cv2.inRange(cv2.imread('link1.png', 1), (200, 200, 200), (255, 255, 255))
    # self.link2 = cv2.inRange(cv2.imread('link2.png', 1), (200, 200, 200), (255, 255, 255))
    # self.link3 = cv2.inRange(cv2.imread('link3.png', 1), (200, 200, 200), (255, 255, 255))   
    
    # The image is loaded as cv_imag

    # Uncomment if you want to save the image
    cv2.imwrite('images/joints.png', cv_image)

    cv2.imshow('window', cv_image)
    cv2.waitKey(3)
    
    jointsData = self.detect_joint_angles(cv_image)
    
    # change te value of self.joint.data to your estimated value from thew images once you have finalized the code
    self.joints = Float64MultiArray()
    self.joints.data = jointsData
    
    try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        self.joints_pub.publish(self.joints)
    except CvBridgeError as e:
        print(e)

    # publish the estimated position of robot end-effector (for lab 3)
    #x_e_image = np.array([0, 0, 0])
    #self.end_effector=Float64MultiArray()
    #self.end_effector.data= x_e_image	

    # send control commands to joints (for lab 3)
    #self.joint1=Float64()
    #self.joint1.data= q_d[0]
    #self.joint2=Float64()
    #self.joint2.data= q_d[1]
    #self.joint3=Float64()
    #self.joint3.data= q_d[2]

    # Publishing the desired trajectory on a topic named trajectory(for lab 3)
    x_d = self.trajectory()    # getting the desired trajectory
    self.trajectory_desired= Float64MultiArray()
    self.trajectory_desired.data=x_d

    # Publish the results
    try: 
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.joints_pub.publish(self.joints)
      #(for lab 3)
      #self.trajectory_pub.publish(self.trajectory_desired)
      #self.end_effector_pub.publish(self.end_effector)
      #self.robot_joint1_pub.publish(self.joint1)
      #self.robot_joint2_pub.publish(self.joint2)
      #self.robot_joint3_pub.publish(self.joint3)
    except CvBridgeError as e:
      print(e)
      
    # Publish the results - the images are published under a topic named "image_topic" and calculated joints angles are published under a topic named "joints_pos"
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.joints_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


