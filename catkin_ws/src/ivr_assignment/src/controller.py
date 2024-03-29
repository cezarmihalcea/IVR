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

        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.effector_pub = rospy.Publisher("/end_effector_position", Float64MultiArray, queue_size=10)

        self.j1 = 1.57
        self.j2 = 1.57
        self.j3 = 1.57
        self.j4 = 1.57

        self.joints = message_filters.Subscriber("joints_pos", Float64MultiArray)
        self.dist = message_filters.Subscriber("square_pos", Float64MultiArray)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.joints, self.dist], 10, 1,                                                     allow_headerless=True)
        self.ts.registerCallback(self.callback)

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
        z_d = float(5)
        print("TRAJECTORY: ", x_d, y_d, z_d)
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

    def control_open(self, joints, dist):
        # estimate time step
        cur_time = rospy.get_time()
        dt = cur_time - self.time_previous_step2
        self.time_previous_step2 = cur_time
        J_inv = np.linalg.pinv(self.calculate_jacobian(joints))  # calculating the pseudo inverse of Jacobian
        # desired trajectory - will replace with desired coordinates of orange floaty square
        #pos_d= self.trajectory()
        pos_d = np.array([dist[0], dist[1], dist[2]])
        #print("Target:", pos_d)
        # estimate derivative of desired trajectory
        self.error_d = (pos_d - self.error)/dt
        self.error = pos_d
        q_d = joints + (dt * np.dot(J_inv, self.error_d.transpose()))  # desired joint angles to follow the trajectory
        return q_d

    def control_closed(self, joints, dist):
        # P gain
        K_p = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])
        # D gain
        K_d = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time
        # robot end-effector position
        pos = self.forward_kinematics(joints)
        # desired trajectory
        pos_d = np.array([dist[0], dist[1], dist[2]])
        #print("Target:", pos_d)
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error) / dt
        # estimate error
        self.error = pos_d - pos
        q = joints  # estimate initial value of joints'
        J_inv = np.linalg.pinv(self.calculate_jacobian(joints))  # calculating the psudeo inverse of Jacobian
        dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p, self.error.transpose())))  # control input (angular velocity of joints)
        q_d = q + (dt * dq_d)  # control input (angular position of joints)
        return q_d

    def callback(self, joints, dist):

        #print([0.0, 0.0, 0.0, 0.0])
        #print(self.forward_kinematics([0.0, 0.0, 0.0, 0.0]))
        #print([0.0, 0.0, 0.0, 0.0])
        #print(self.forward_kinematics([0.0, 1.57, 1.57, 1.57]))
        #print([-2.2, -1.0, -0.1, -0.9])
        #print(self.forward_kinematics([2.2, 1.0, 0.1, 0.9]))
        #print([1.5, 0.1, 0.1, 0.1])
        #print(self.forward_kinematics([1.5, 0.1, 0.1, 0.1]))
        #print([3.1, 1.5, 1.0, 1.3])
        #print(self.forward_kinematics([3.1, 1.5, 1.0, 1.3]))
        #print([0.1, 0.4, 0.6, 0.5])
        #print(self.forward_kinematics([0.1, 0.4, 0.6, 0.5]))
        #print([0.4, 1.2, 1.4, -0.7])
        #print(self.forward_kinematics([0.4, 1.2, 1.4, -0.7]))
        #print([0.6, 0.7, -0.8, 0.9])
        #print(self.forward_kinematics([0.6, 0.7, -0.8, 0.9]))
        #print([1.0, 1.0, 1.0, 1.0])
        #print(self.forward_kinematics([1.0, 1.0, 1.0, 1.0]))
        ##print([0.2, -1.1, 0.7, 0.0])
        #print(self.forward_kinematics([0.2, -1.1, 0.7, 0.0]))
        #print([0.0, 1.0, 0.9, -0.3])
        #print(self.forward_kinematics([0.0, 1.0, 0.9, -0.3]))

        #print("Current time:", rospy.get_time())

        #print(self.j1, self.j2, self.j3, self.j4)
        #print("Current joint positions:", joints.data)

        #print("Calculated forward kinematics:", self.forward_kinematics(joints.data))

        end_pos = Float64MultiArray()
        end_pos.data = self.forward_kinematics(joints.data)
        self.effector_pub.publish(end_pos)
        #print(self.forward_kinematics([self.j1,self.j2,self.j3,self.j4]))

        self.joint_moves = self.control_closed(joints.data, dist.data)
        #self.joint_moves = self.control_open([self.j1,self.j2,self.j3,self.j4])

        #print("Joint movements:", self.joint_moves)
        #print ""

        try:
            self.robot_joint1_pub.publish(self.joint_moves[0])
            #self.j1 = self.joint_moves[0]
            self.robot_joint2_pub.publish(self.joint_moves[1])
            #self.j2 = self.joint_moves[1]
            self.robot_joint3_pub.publish(self.joint_moves[2])
            #self.j3 = self.joint_moves[2]
            self.robot_joint4_pub.publish(self.joint_moves[3])
            #self.j4 = self.joint_moves[3]
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
