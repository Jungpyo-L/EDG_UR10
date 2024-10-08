#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String

import moveit_commander
from geometry_msgs.msg import PoseStamped
# import endeffectorOffset as eff_offsetCal

import tf



def robotStatePublisher():
    pub = rospy.Publisher('endEffectorPose', PoseStamped, queue_size=10)
    rospy.init_node('robotStatePublisher', anonymous=True)

    listener = tf.TransformListener()

    rate = rospy.Rate(30)
    effPoseMsg = PoseStamped()
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))
            # (trans,rot) = listener.lookupTransform('/base_link', '/wrist_3_link', rospy.Time(0))
            
            effPoseMsg.pose.position.x = trans[0]
            effPoseMsg.pose.position.y = trans[1]
            effPoseMsg.pose.position.z = trans[2]

            effPoseMsg.pose.orientation.x = rot[0]
            effPoseMsg.pose.orientation.y = rot[1]
            effPoseMsg.pose.orientation.z = rot[2]
            effPoseMsg.pose.orientation.w = rot[3]

            effPoseMsg.header.stamp= rospy.Time.now()

            pub.publish(effPoseMsg)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        robotStatePublisher()
    except rospy.ROSInterruptException:
        pass
