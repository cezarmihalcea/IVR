#!/usr/bin/env python

import sys
import cv2
import rospy
import numpy as np
import message_filters
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class controller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        self.robot_joint1_pub = rospy.Publisher("/robot/joint1positioncontroller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint1positioncontroller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint1positioncontroller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint1positioncontroller/command", Float64, queue_size=10)

        self.joints = message_filters.Subscriber("joints_pos", Float64MultiArray)

        self.joints.registerCallback(self.callback)

        self.bridge = CvBridge()
        self.time_trajectory = rospy.get_time()
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')

        self.joint_moves = np.array([0.0,0.0,0.0,0.0], dtype='float64')

    # Define a circular trajectory
    def trajectory(self):
        # get current time
        cur_time = np.array([rospy.get_time() - self.time_trajectory])
        x_d = float(6 * np.cos(cur_time * np.pi / 100))
        y_d = float(6 + np.absolute(1.5 * np.sin(cur_time * np.pi / 100)))
        z_d = float(2.5)
        return np.array([x_d, y_d, z_d])

    def forward_kinematics(self, joints):
        end_effector = np.array([(-3*np.cos(joints[0]+joints[1]+joints[2])+3*np.cos(joints[0]-joints[1]+joints[2])-3*np.cos(joints[0]+joints[1]-joints[2])+3*np.cos(joints[0]-joints[1]-joints[2])-2*np.cos(joints[0]+joints[1]+joints[3])-2*np.cos(joints[0]-joints[1]+joints[3])-np.cos(joints[0]+joints[1]+joints[2]+joints[3])+np.cos(joints[0]-joints[1]+joints[2]+joints[3])-np.cos(joints[0]+joints[1]-joints[2]+joints[3])+np.cos(joints[0]-joints[1]-joints[2]+joints[3])+2*np.cos(joints[0]+joints[1]-joints[3])+2*np.cos(joints[0]-joints[1]-joints[3])-np.cos(joints[0]+joints[1]+joints[2]-joints[3])+np.cos(joints[0]-joints[1]+joints[2]-joints[3])-np.cos(joints[0]+joints[1]-joints[2]-joints[3])+np.cos(joints[0]-joints[1]-joints[2]-joints[3])+6*np.sin(joints[0]+joints[2])-6*np.sin(joints[0]-joints[2])+2*np.sin(joints[0]+joints[2]+joints[3])-2*np.sin(joints[0]-joints[2]+joints[3])+2*np.sin(joints[0]+joints[2]-joints[3])-2*np.sin(joints[0]-joints[2]-joints[3]))/4,
                                 (-6*np.cos(joints[0]+joints[2])+6*np.cos(joints[0]-joints[2])-2*np.cos(joints[0]+joints[2]+joints[3])+2*np.cos(joints[0]-joints[2]+joints[3])-2*np.cos(joints[0]+joints[2]-joints[3])+2*np.cos(joints[0]-joints[2]-joints[3])-3*np.sin(joints[0]+joints[1]+joints[2])+3*np.sin(joints[0]-joints[1]+joints[2])-3*np.sin(joints[0]+joints[1]-joints[2])+3*np.sin(joints[0]-joints[1]-joints[2])-2*np.sin(joints[0]+joints[1]+joints[3])-2*np.sin(joints[0]-joints[1]+joints[3])-np.sin(joints[0]+joints[1]+joints[2]+joints[3])+np.sin(joints[0]-joints[1]+joints[2]+joints[3])-np.sin(joints[0]+joints[1]-joints[2]+joints[3])+np.sin(joints[0]-joints[1]-joints[2]+joints[3])+2*np.sin(joints[0]+joints[1]-joints[3])+2*np.sin(joints[0]-joints[1]-joints[3])-np.sin(joints[0]+joints[1]+joints[2]-joints[3])+np.sin(joints[0]-joints[1]+joints[2]-joints[3])-np.sin(joints[0]+joints[1]-joints[2]-joints[3])+np.sin(joints[0]-joints[1]-joints[2]-joints[3]))/4,
                                 (3*np.cos(joints[1]+joints[2])+3*np.cos(joints[1]-joints[2])+2*np.cos(joints[1]+joints[3])+np.cos(joints[1]+joints[2]+joints[3])+np.cos(joints[1]-joints[2]+joints[3])-2*np.cos(joints[1]-joints[3])+np.cos(joints[1]+joints[2]-joints[3])+np.cos(joints[1]-joints[2]-joints[3])+4)/2])
        return end_effector

    def calculate_jacobian(self, joints):
        jacobian = np.array([[(-3*-np.sin(joints[0]+joints[1]+joints[2])+3*-np.sin(joints[0]-joints[1]+joints[2])-3*-np.sin(joints[0]+joints[1]-joints[2])+3*-np.sin(joints[0]-joints[1]-joints[2])-2*-np.sin(joints[0]+joints[1]+joints[3])-2*-np.sin(joints[0]-joints[1]+joints[3])+np.sin(joints[0]+joints[1]+joints[2]+joints[3])+-np.sin(joints[0]-joints[1]+joints[2]+joints[3])+np.sin(joints[0]+joints[1]-joints[2]+joints[3])+-np.sin(joints[0]-joints[1]-joints[2]+joints[3])+2*-np.sin(joints[0]+joints[1]-joints[3])+2*-np.sin(joints[0]-joints[1]-joints[3])+np.sin(joints[0]+joints[1]+joints[2]-joints[3])+-np.sin(joints[0]-joints[1]+joints[2]-joints[3])+np.sin(joints[0]+joints[1]-joints[2]-joints[3])+-np.sin(joints[0]-joints[1]-joints[2]-joints[3])+6*np.cos(joints[0]+joints[2])-6*np.cos(joints[0]-joints[2])+2*np.cos(joints[0]+joints[2]+joints[3])-2*np.cos(joints[0]-joints[2]+joints[3])+2*np.cos(joints[0]+joints[2]-joints[3])-2*np.cos(joints[0]-joints[2]-joints[3]))/4,
                              (-3*-np.sin(joints[0]+joints[1]+joints[2])+3*-np.sin(joints[0]-joints[1]+joints[2])-3*-np.sin(joints[0]+joints[1]-joints[2])+3*-np.sin(joints[0]-joints[1]-joints[2])-2*-np.sin(joints[0]+joints[1]+joints[3])-2*-np.sin(joints[0]-joints[1]+joints[3])+np.sin(joints[0]+joints[1]+joints[2]+joints[3])+-np.sin(joints[0]-joints[1]+joints[2]+joints[3])+np.sin(joints[0]+joints[1]-joints[2]+joints[3])+-np.sin(joints[0]-joints[1]-joints[2]+joints[3])+2*-np.sin(joints[0]+joints[1]-joints[3])+2*-np.sin(joints[0]-joints[1]-joints[3])+np.sin(joints[0]+joints[1]+joints[2]-joints[3])+-np.sin(joints[0]-joints[1]+joints[2]-joints[3])+np.sin(joints[0]+joints[1]-joints[2]-joints[3])+-np.sin(joints[0]-joints[1]-joints[2]-joints[3]))/4,
                              (-3*-np.sin(joints[0]+joints[1]+joints[2])+3*-np.sin(joints[0]-joints[1]+joints[2])-3*-np.sin(joints[0]+joints[1]-joints[2])+3*-np.sin(joints[0]-joints[1]-joints[2]) +np.sin(joints[0]+joints[1]+joints[2]+joints[3])+-np.sin(joints[0]-joints[1]+joints[2]+joints[3])+np.sin(joints[0]+joints[1]-joints[2]+joints[3])+-np.sin(joints[0]-joints[1]-joints[2]+joints[3]) +np.sin(joints[0]+joints[1]+joints[2]-joints[3])+-np.sin(joints[0]-joints[1]+joints[2]-joints[3])+np.sin(joints[0]+joints[1]-joints[2]-joints[3])+-np.sin(joints[0]-joints[1]-joints[2]-joints[3])+6*np.cos(joints[0]+joints[2])-6*np.cos(joints[0]-joints[2])+2*np.cos(joints[0]+joints[2]+joints[3])-2*np.cos(joints[0]-joints[2]+joints[3])+2*np.cos(joints[0]+joints[2]-joints[3])-2*np.cos(joints[0]-joints[2]-joints[3]))/4,
                              (-2*-np.sin(joints[0]+joints[1]+joints[3])-2*-np.sin(joints[0]-joints[1]+joints[3])+np.sin(joints[0]+joints[1]+joints[2]+joints[3])+-np.sin(joints[0]-joints[1]+joints[2]+joints[3])+np.sin(joints[0]+joints[1]-joints[2]+joints[3])+-np.sin(joints[0]-joints[1]-joints[2]+joints[3])+2*-np.sin(joints[0]+joints[1]-joints[3])+2*-np.sin(joints[0]-joints[1]-joints[3])+np.sin(joints[0]+joints[1]+joints[2]-joints[3])+-np.sin(joints[0]-joints[1]+joints[2]-joints[3])+np.sin(joints[0]+joints[1]-joints[2]-joints[3])+-np.sin(joints[0]-joints[1]-joints[2]-joints[3])+2*np.cos(joints[0]+joints[2]+joints[3])-2*np.cos(joints[0]-joints[2]+joints[3])+2*np.cos(joints[0]+joints[2]-joints[3])-2*np.cos(joints[0]-joints[2]-joints[3]))/4],
                             [(-6*-np.sin(joints[0]+joints[2])+6*-np.sin(joints[0]-joints[2])-2*-np.sin(joints[0]+joints[2]+joints[3])+2*-np.sin(joints[0]-joints[2]+joints[3])-2*-np.sin(joints[0]+joints[2]-joints[3])+2*-np.sin(joints[0]-joints[2]-joints[3])-3*np.cos(joints[0]+joints[1]+joints[2])+3*np.cos(joints[0]-joints[1]+joints[2])-3*np.cos(joints[0]+joints[1]-joints[2])+3*np.cos(joints[0]-joints[1]-joints[2])-2*np.cos(joints[0]+joints[1]+joints[3])-2*np.cos(joints[0]-joints[1]+joints[3])-np.cos(joints[0]+joints[1]+joints[2]+joints[3])+np.cos(joints[0]-joints[1]+joints[2]+joints[3])-np.cos(joints[0]+joints[1]-joints[2]+joints[3])+np.cos(joints[0]-joints[1]-joints[2]+joints[3])+2*np.cos(joints[0]+joints[1]-joints[3])+2*np.cos(joints[0]-joints[1]-joints[3])-np.cos(joints[0]+joints[1]+joints[2]-joints[3])+np.cos(joints[0]-joints[1]+joints[2]-joints[3])-np.cos(joints[0]+joints[1]-joints[2]-joints[3])+np.cos(joints[0]-joints[1]-joints[2]-joints[3]))/4,
                              (-3*np.cos(joints[0]+joints[1]+joints[2])+3*np.cos(joints[0]-joints[1]+joints[2])-3*np.cos(joints[0]+joints[1]-joints[2])+3*np.cos(joints[0]-joints[1]-joints[2])-2*np.cos(joints[0]+joints[1]+joints[3])-2*np.cos(joints[0]-joints[1]+joints[3])-np.cos(joints[0]+joints[1]+joints[2]+joints[3])+np.cos(joints[0]-joints[1]+joints[2]+joints[3])-np.cos(joints[0]+joints[1]-joints[2]+joints[3])+np.cos(joints[0]-joints[1]-joints[2]+joints[3])+2*np.cos(joints[0]+joints[1]-joints[3])+2*np.cos(joints[0]-joints[1]-joints[3])-np.cos(joints[0]+joints[1]+joints[2]-joints[3])+np.cos(joints[0]-joints[1]+joints[2]-joints[3])-np.cos(joints[0]+joints[1]-joints[2]-joints[3])+np.cos(joints[0]-joints[1]-joints[2]-joints[3]))/4,
                              (-6*-np.sin(joints[0]+joints[2])+6*-np.sin(joints[0]-joints[2])-2*-np.sin(joints[0]+joints[2]+joints[3])+2*-np.sin(joints[0]-joints[2]+joints[3])-2*-np.sin(joints[0]+joints[2]-joints[3])+2*-np.sin(joints[0]-joints[2]-joints[3])-3*np.cos(joints[0]+joints[1]+joints[2])+3*np.cos(joints[0]-joints[1]+joints[2])-3*np.cos(joints[0]+joints[1]-joints[2])+3*np.cos(joints[0]-joints[1]-joints[2]) -np.cos(joints[0]+joints[1]+joints[2]+joints[3])+np.cos(joints[0]-joints[1]+joints[2]+joints[3])-np.cos(joints[0]+joints[1]-joints[2]+joints[3])+np.cos(joints[0]-joints[1]-joints[2]+joints[3]) -np.cos(joints[0]+joints[1]+joints[2]-joints[3])+np.cos(joints[0]-joints[1]+joints[2]-joints[3])-np.cos(joints[0]+joints[1]-joints[2]-joints[3])+np.cos(joints[0]-joints[1]-joints[2]-joints[3]))/4,
                              (-2*-np.sin(joints[0]+joints[2]+joints[3])+2*-np.sin(joints[0]-joints[2]+joints[3])-2*-np.sin(joints[0]+joints[2]-joints[3])+2*-np.sin(joints[0]-joints[2]-joints[3]) -2*np.cos(joints[0]+joints[1]+joints[3])-2*np.cos(joints[0]-joints[1]+joints[3])-np.cos(joints[0]+joints[1]+joints[2]+joints[3])+np.cos(joints[0]-joints[1]+joints[2]+joints[3])-np.cos(joints[0]+joints[1]-joints[2]+joints[3])+np.cos(joints[0]-joints[1]-joints[2]+joints[3])+2*np.cos(joints[0]+joints[1]-joints[3])+2*np.cos(joints[0]-joints[1]-joints[3])-np.cos(joints[0]+joints[1]+joints[2]-joints[3])+np.cos(joints[0]-joints[1]+joints[2]-joints[3])-np.cos(joints[0]+joints[1]-joints[2]-joints[3])+np.cos(joints[0]-joints[1]-joints[2]-joints[3]))/4],
                             [0,
                              (3*-np.sin(joints[1]+joints[2])+3*-np.sin(joints[1]-joints[2])+2*-np.sin(joints[1]+joints[3])+-np.sin(joints[1]+joints[2]+joints[3])+-np.sin(joints[1]-joints[2]+joints[3])-2*-np.sin(joints[1]-joints[3])+-np.sin(joints[1]+joints[2]-joints[3])+-np.sin(joints[1]-joints[2]-joints[3])+4)/2,
                              (3*-np.sin(joints[1]+joints[2])+3*-np.sin(joints[1]-joints[2]) +-np.sin(joints[1]+joints[2]+joints[3])+-np.sin(joints[1]-joints[2]+joints[3]) +-np.sin(joints[1]+joints[2]-joints[3])+-np.sin(joints[1]-joints[2]-joints[3])+4)/2,
                              (2*-np.sin(joints[1]+joints[3])+-np.sin(joints[1]+joints[2]+joints[3])+-np.sin(joints[1]-joints[2]+joints[3])-2*-np.sin(joints[1]-joints[3])+-np.sin(joints[1]+joints[2]-joints[3])+-np.sin(joints[1]-joints[2]-joints[3])+4)/2]])
        return jacobian

    def control_open(self, joints):
        # estimate time step
        cur_time = rospy.get_time()
        dt = cur_time - self.time_previous_step2
        self.time_previous_step2 = cur_time
        J_inv = np.linalg.pinv(self.calculate_jacobian(joints))  # calculating the pseudo inverse of Jacobian
        # desired trajectory - will replace with desired coordinates of orange floaty square
        pos_d= self.trajectory()
        # estimate derivative of desired trajectory
        self.error_d = (pos_d - self.error)/dt
        self.error = pos_d
        q_d = joints + (dt * np.dot(J_inv, self.error_d.transpose()))  # desired joint angles to follow the trajectory
        return q_d

    def callback(self, joints):
        self.joint_moves = self.control_open(joints.data)

        try:
            self.robot_joint1_pub.publish(self.joint_moves[0])
            self.robot_joint2_pub.publish(self.joint_moves[1])
            self.robot_joint3_pub.publish(self.joint_moves[2])
            self.robot_joint4_pub.publish(self.joint_moves[3])
        except CvBridgeError as e:
            print(e)


def main(args):
    c = controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
