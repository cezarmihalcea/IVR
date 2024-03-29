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

def detect_green(image):
  mask = cv2.inRange(image, (0,100,0), (0,255,0))
  
  kernel = np.ones((5, 5), np.uint8)
  mask = cv2.dilate(mask, kernel, iterations = 3)
  if np.sum(mask) != 0:
    m = cv2.moments(mask)
    
    cx = int(m['m10'] / m['m00'])
    cy = int(m['m01'] / m['m00'])
    
    return np.array([cx, cy])
  else:
    return np.array([0, 0])

def detect_red(image):
  mask = cv2.inRange(image, (0,0,100), (0,0,255))
  
  kernel = np.ones((5, 5), np.uint8)
  mask = cv2.dilate(mask, kernel, iterations = 3)
  if np.sum(mask) != 0:
    m = cv2.moments(mask)
    
    cx = int(m['m10'] / m['m00'])
    cy = int(m['m01'] / m['m00'])
    
    return np.array([cx, cy])
  else:
    return np.array([0, 0])

def detect_blue(image):
  mask = cv2.inRange(image, (100,0,0), (255,0,0))
  
  kernel = np.ones((5, 5), np.uint8)
  mask = cv2.dilate(mask, kernel, iterations = 3)
  if np.sum(mask) != 0:
    m = cv2.moments(mask)
    
    cx = int(m['m10'] / m['m00'])
    cy = int(m['m01'] / m['m00'])
    
    return np.array([cx, cy])
  else:
    return np.array([0, 0])

def detect_yellow(image):
  mask = cv2.inRange(image, (0,100,100), (0,255,255))
  
  kernel = np.ones((5, 5), np.uint8)
  mask = cv2.dilate(mask, kernel, iterations = 3)
  if np.sum(mask) != 0:
    m = cv2.moments(mask)
    
    cx = int(m['m10'] / m['m00'])
    cy = int(m['m01'] / m['m00'])
    
    return np.array([cx, cy])
  else:
    return np.array([0, 0])

class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge() 
      
  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
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


