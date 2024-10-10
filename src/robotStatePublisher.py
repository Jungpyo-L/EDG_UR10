#!/usr/bin/env python
# this is for the robot state publisher
# It uses tf to get the end effector pose and publishes it to the topic 'endEffectorPose'
# In order to match the coordinate system of the RTDE, the transformation is used

import sys
import rospy
import numpy as np
from std_msgs.msg import String

import moveit_commander
from geometry_msgs.msg import PoseStamped
from helperFunction.utils import create_transform_matrix
from helperFunction.transformation_matrix import get_Tmat_from_PositionQuat, get_ObjectPoseStamped_from_T, getPoseObj
from scipy.spatial.transform import Rotation as R

import rtde_receive

import tf

def robotStatePublisher():
    pub = rospy.Publisher('endEffectorPose', PoseStamped, queue_size=10)
    rospy.init_node('robotStatePublisher', anonymous=True)

    rtde_r = rtde_receive.RTDEReceiveInterface("10.0.0.1", 125)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        try:

            TCPPose = rtde_r.getActualTCPPose()
            Position = [TCPPose[0], TCPPose[1], TCPPose[2]]
            r = R.from_rotvec(np.array([TCPPose[3], TCPPose[4], TCPPose[5]]))
            pose = getPoseObj(Position, r.as_quat())
            pose.header.stamp = rospy.Time.now()
            pub.publish(pose)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        robotStatePublisher()
    except rospy.ROSInterruptException:
        pass
