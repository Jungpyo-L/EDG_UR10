#!/usr/bin/env python
# Created by Nimran Shergill (April 21 2025). 
# This is a file for combining planar motion with rotational motion.
# This file is part of the EDG UR10 project.

# Imports
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
import string
import copy

from netft_utils.srv import *
from edg_ur10.srv import *

from helperFunction.rtde_helper import rtdeHelp
from helperFunction.utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix, normalize, hat


from datetime import datetime
import numpy as np
import time
import scipy
import pickle

from netft_utils.srv import *
from suction_cup.srv import *
from std_msgs.msg import String
from std_msgs.msg import Int8
import geometry_msgs.msg

from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp

def main(args):
  deg2rad = np.pi / 180.0

  np.set_printoptions(precision=4)

  # controller node 
  rospy.init_node('edg_experiment')

  # Setup helper functions
  FT_help = FT_CallbackHelp() # it deals with subscription.
  rospy.sleep(0.5)
  rtde_help = rtdeHelp(125)
  rospy.sleep(0.5)
  file_help = fileSaveHelp()
  adpt_help = adaptMotionHelp(d_w = 15,d_lat = 10e-3, d_z= 5e-3)
  rospy.sleep(0.5)

  # Set the synchronization Publisher
  syncPub = rospy.Publisher('sync', Int8, queue_size=1)
  # Set the data_logger
  print("Wait for the data_logger to be enabled")
  rospy.wait_for_service('data_logging')

  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable) 
  dataLoggerEnable(False) # reset Data Logger just in case
  rospy.sleep(1)
  file_help.clearTmpFolder()        # clear the temporary folder
  datadir = file_help.ResultSavingDirectory

  #################################################
  # DEFINE POSES FOR RESETTING TRIALS             #
  # Note: Check the pose of the robot before running code. 
  # Use View feature and make sure the nuymbers below won't cause the robot to hit anything.
  #################################################
  ## Rise out of media. No rotation.
  currentPose = rtde_help.getCurrentPose()
  PositionA = [currentPose.pose.position.x, currentPose.pose.position.y, 0.350]
  OrientationA = [currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w]
  PoseA = rtde_help.getPoseObj(PositionA, OrientationA)

  # Pose B has to be defined relative to A so it is defined during the motion sequence

  # We descend into media. No rotation. 
  PositionC = [0.200, -0.230, 0.300] # approx 7 cm below surface of grains, edit to 0.270
  OrientationC = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi,'sxyz') # not moving it from the previous transformation
  PoseC = rtde_help.getPoseObj(PositionC, OrientationC) 

    # Pose D has to be defined relative to C so it is defined during the motion sequence
  #######################################################################
  ################ REACH STARTING POSITION ########################
  try:
    # POSE A
    input("Press <Enter> to go to PoseA")
    rtde_help.goToPose(PoseA) 
    rospy.sleep(1)

    # POSE B
    input("Press <Enter> to go to PoseB")
    currentPose = rtde_help.getCurrentPose()
    PositionB = [0.200, -0.230, currentPose.pose.position.z] # change the first two parameters to be the "beginning of the tank"
    OrientationB = tf.transformations.quaternion_from_euler(np.pi, 0,-np.pi,'sxyz') #static (s) rotating (r)
    #   Note the new coordinates: x is pointing to us, y is pointing to the left, and z is pointing down.
    PoseB = rtde_help.getPoseObj(PositionB, OrientationB)
    rtde_help.goToPose(PoseB) 
    rospy.sleep(1)

    # POSE C
    input("Press <Enter> to go to PoseC")
    rtde_help.goToPose(PoseC) 
    rospy.sleep(1)
    
    # POSE D
    print('\n')
    input("Press <Enter> to go to PoseD")
    currentPose = rtde_help.getCurrentPose()
    PositionD = [0.240, currentPose.pose.position.y, currentPose.pose.position.z] # approx 8 cm below surface of grains
    OrientationD = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi,'sxyz') # not moving it from the previous transformation 
    PoseD = rtde_help.getPoseObj(PositionD, OrientationD) 
    rtde_help.goToPose(PoseD)
    rospy.sleep(1)

    # ZERO GRAVITY AND OTHER FORCES
    FT_help.setNowAsBias() # offset the force sensor, zeros gravity and other forces
    args.ForceOffset1 = [FT_help.offSetFx, FT_help.offSetFy, FT_help.offSetFz, FT_help.offSetTx, FT_help.offSetTy, FT_help.offSetTz]

    input("Press <Enter> to snout motion sequence with horizontal motion + rotations")
    dataLoggerEnable(True)
    rospy.sleep(0.5) # default is 0.5

    ################################ INITIATE MOTION SEQUENCE ################################
    # Initializing parameters
    T_move = np.eye(4) 
    overall_angle = 0 
    currentPose = rtde_help.getCurrentPose()
    T_start = adpt_help.get_Tmat_from_Pose(currentPose) # get the transformation matrix from the current pose
    R_start = T_start[:3,:3] # get the rotation matrix from the transformation matrix
    FT_help.setNowAsBias() # not needed yet, but will be used later

    currentPose = rtde_help.getCurrentPose() # get the current pose after the motion

    while overall_angle < 10: 
      # Task: Move forward and rotate to 5 degrees along the way
      # Define some conditional to stop doing the joint motion
      # To create motion:
      # Define motion matrix
      adpt_help.dw = 0.05 # slowing down the rotation speed
      T_rot_step = adpt_help.get_Tmat_RotateInY(direction=1) # rotate in y direction, about the 
      T_horizontal_step = adpt_help.get_Tmat_TranlateInX(direction=-1) # move in x direction 
      T_vertical_step = adpt_help.get_Tmat_TranlateInZ(direction=1) # move in z direction, not used here but can be used later
      # print("T_rot_step: ", T_rot_step)
      # print("T_horizontal_step: ", T_horizontal_step)
      # print("T_vertical_step: ", T_vertical_step)

      # Does not consider motion strictly in the global frame, but rather in the local frame of the robot
      # both T_move and T_move2 work
      T_move = T_horizontal_step @ T_rot_step # T_move = Translation * Rotation * Scaling
      T_trans = T_horizontal_step @ T_vertical_step
      print("T_trans: ", T_trans)
      print("T_rot_step: ", T_rot_step)
      T_move2 = T_trans @ T_rot_step # T_move = Translation * Rotation * Scaling
      print("T_move2: ", T_move2)

      # TODO: projecting back to the global frame


      targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move2, currentPose) # get the target pose from the transformation matrix and the current pose
      rtde_help.goToPoseAdaptive(targetPose, time=0.5) # move to the target pose
      
      
      currentPose = rtde_help.getCurrentPose() # get the current pose after the motion
      T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
      T_overall = np.linalg.inv(T_start) @ T_curr 
      overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi 
      if T_overall[2, 0] > 0:  
          overall_angle = -overall_angle
      print("Current angle: ", overall_angle)
      print('Current x position: ', currentPose.pose.position.x)
    #rtde_help.stopAtCurrPoseAdaptive() # stop at the current pose



  except rospy.ROSInterruptException:
        return
  except KeyboardInterrupt:
        return  

if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  # parser.add_argument('--timeLimit', type=float, help='time limit for the adaptive motion', default= 5)
  # parser.add_argument('--pathlLimit', type=float, help='path-length limit for the adaptive motion (m)', default= 0.01)
  parser.add_argument('--normalForce', type=float, help='normal force threshold', default=0.5)
  parser.add_argument('--beta', type=int, help='beta angle of wedge', default= 0) # wrote zero so I realize I didn't change this parameter
  parser.add_argument('--trialNum', type=int, help='Trial number', default= 1)
  args = parser.parse_args()    

  main(args)
