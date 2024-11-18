#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: Oct.08.2024
# Last update: Oct.10.2024
# Description: This script is primarily for basic robot control using UR10e robot. 
# It first moves to initial position (position A), then it move position A (poseA) to B (poseB) and rotate by 45 degrees in y-axis (poseC).

# imports
try:
  import rospy
  import tf
  ros_enabled = True
except:
  print('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
  ros_enabled = False

from calendar import month_abbr
import os, sys
import numpy as np
import copy

from netft_utils.srv import *
from edg_ur10.srv import *

from helperFunction.rtde_helper import rtdeHelp

def main():

  deg2rad = np.pi / 180.0

  np.set_printoptions(precision=4)

  # controller node
  rospy.init_node('edg_experiment')

  # Setup helper functions
  rtde_help = rtdeHelp(125)
  rospy.sleep(0.5)

  # Set the TCP offset and calibration matrix (ex, suction cup: 0.150, ATI_default: 0.464)
  # You can set the TCP offset here, but it is recommended to set it in the UR program.
  # If you set it here, endEffectorPose will be different from the actual pose.
  # rtde_help.setTCPoffset([0, 0, 0.464, 0, 0, 0])
  # rospy.sleep(0.2)

  # Set the pose A
  positionA = [0.520, -0.200, 0.40]
  orientationA = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA, orientationA)

  # Set the pose B
  positionB = [0.620, -0.100, 0.50]
  orientationA = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseB = rtde_help.getPoseObj(positionB, orientationA)

  # Set the pose C
  positionB = [0.620, -0.100, 0.50]
  orientationB  = tf.transformations.quaternion_from_euler(np.pi + 45*deg2rad,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseC = rtde_help.getPoseObj(positionB, orientationB)

  # try block so that we can have a keyboard exception
  try:

    input("Press <Enter> to go to pose A")
    rtde_help.goToPose(poseA)
    rospy.sleep(1)
    print("poseA: ", rtde_help.getCurrentPose())

    input("Press <Enter> to go to pose B")
    rtde_help.goToPose(poseB)
    rospy.sleep(1)
    print("poseB: ", rtde_help.getCurrentPose())

    input("Press <Enter> to go to pose C")
    rtde_help.goToPose(poseC)
    rospy.sleep(1)
    print("poseC: ", rtde_help.getCurrentPose())
    

    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':
  main()
