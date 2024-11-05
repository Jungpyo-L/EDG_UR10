#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: Nov.04.2024
# Last update: Nov.04.2024
# Description: This script is primarily for adaptive robot control using UR10e robot. 
# It first moves to initial position (position A), then it move position A (poseA) to B (poseB) and preform adaptive robot control.
# It basically moves to the right and move up and down depending on the force feedback. If the force is negative, it moves up, otherwise it moves down.

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
import time

from netft_utils.srv import *
from edg_ur10.srv import *


from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.rtde_helper import rtdeHelp
from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.fileSaveHelper import fileSaveHelp

def main(args):

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
  adpt_help = adaptMotionHelp(d_w = 1,d_lat = 10e-3, d_z= 5e-3)

  # Set the TCP offset and calibration matrix (ex, suction cup: 0.150, ATI_default: 0.464)
  # You can set the TCP offset here, but it is recommended to set it in the UR program.
  # If you set it here, endEffectorPose will be different from the actual pose.
  # rtde_help.setTCPoffset([0, 0, 0.464, 0, 0, 0])
  # rospy.sleep(0.2)

  # Set force threshold
  F_normalThres = args.normalForce

  # Set the data_logger
  print("Wait for the data_logger to be enabled")
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable) #*
  dataLoggerEnable(False) # reset Data Logger just in case
  rospy.sleep(1)
  file_help.clearTmpFolder()        # clear the temporary folder
  datadir = file_help.ResultSavingDirectory


  # Set the start pose
  startPosition = [0.520, -0.200, 0.40]
  startOrientation = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  # Note the new coordinates: x is pointing to us, y is pointing to the left, and z is pointing down.
  startPose = rtde_help.getPoseObj(startPosition, startOrientation)

  # try block so that we can have a keyboard exception
  try:

    input("Press <Enter> to go to startPose")
    rtde_help.goToPose(startPose)
    rospy.sleep(1)

    # offset the force sensor
    FT_help.setNowAsBias()


    input("Press <Enter> to adaptive robot control")
    startTime = time.time()
    

    # loop for adaptive robot control for args.timeLimit seconds
    while (time.time() - startTime) < args.timeLimit:
      # Get the current pose
      currentPose = rtde_help.getCurrentPose()

      # Get the next pose
      # lateral
      T_lat = adpt_help.get_Tmat_TranlateInY(direction = -1)
      # rotation
      T_rot = np.eye(4)
      # normal
      F_normal = FT_help.averageFz_noOffset
      T_normal = adpt_help.get_Tmat_axialMove(F_normal, args.normalForce)
      T_move = T_lat @ T_normal @ T_rot
      currPose = rtde_help.getCurrentPose()
      targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currPose)

      # Move to the next pose
      rtde_help.goToPoseAdaptive(targetPose, time = 0.1)


    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--timeLimit', type=float, help='time limit for the adaptive motion', default= 5)
  parser.add_argument('--pathlLimit', type=float, help='path-length limit for the adaptive motion (m)', default= 0.1)
  parser.add_argument('--normalForce', type=float, help='normal force threshold', default= 0.5)  
  args = parser.parse_args()    

  main(args)
