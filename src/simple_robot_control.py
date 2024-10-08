#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: Oct.08.2024
# Last update: Oct.08.2024
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

from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp

def main():

  deg2rad = np.pi / 180.0

  np.set_printoptions(precision=4)

  # controller node
  rospy.init_node('edg_experiment')

  # Setup helper functions
  FT_help = FT_CallbackHelp() # it deals with subscription.
  rospy.sleep(0.5)
  rtde_help = rtdeHelp(125, speed=0.1 , acc= 0.1)
  rospy.sleep(0.5)
  file_help = fileSaveHelp()
  rospy.sleep(0.5)

  # Set the TCP offset and calibration matrix
  rtde_help.setTCPoffset([0, 0, 0.0, 0, 0, 0])
  rospy.sleep(0.2)

  # Set the pose A
  positionA = [-0.520, .300, 0.20]
  orientationA = tf.transformations.quaternion_from_euler(np.pi,0,np.pi/2,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA, orientationA)

  # Set the pose B
  positionB = [-0.620, .200, 0.30]
  orientationA = tf.transformations.quaternion_from_euler(np.pi,0,np.pi/2,'sxyz') #static (s) rotating (r)
  poseB = rtde_help.getPoseObj(positionB, orientationA)

  # Set the pose C
  positionB = [-0.620, .200, 0.30]
  orientationB  = tf.transformations.quaternion_from_euler(np.pi + 45*deg2rad,0,np.pi/2,'sxyz') #static (s) rotating (r)
  poseC = rtde_help.getPoseObj(positionB, orientationB)
  

  # try block so that we can have a keyboard exception
  try:

    input("Press <Enter> to go to pose A")
    rtde_help.goToPose(poseA)
    rospy.sleep(1)

    input("Press <Enter> to go to pose B")
    rtde_help.goToPose(poseB)
    rospy.sleep(1)

    input("Press <Enter> to go to pose C")
    rtde_help.goToPose(poseC)
    rospy.sleep(1)
    

    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':
  main()
