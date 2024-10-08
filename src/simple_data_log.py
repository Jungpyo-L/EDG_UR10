#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: Oct.08.2024
# Last update: Oct.08.2024
# Description: This script is primarily for basic robot control using UR10e robot. 
# It first moves to initial position (position A), then it move position A to B and rotate by 45 degrees in y-axis.

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
import string
from helperFunction.utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix, normalize, hat


from datetime import datetime
import pandas as pd
import re
import subprocess
import numpy as np
import copy
import time
import scipy
import pickle
import shutil
from scipy.io import savemat
from scipy.spatial.transform import Rotation as sciRot


from netft_utils.srv import *
from suction_cup.srv import *
from std_msgs.msg import String
from std_msgs.msg import Int8
import geometry_msgs.msg
import tf
import cv2
from scipy import signal

from math import pi, cos, sin, floor

from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp


def main(args):

  deg2rad = np.pi / 180.0

  SYNC_RESET = 0
  SYNC_START = 1
  SYNC_STOP = 2

  FT_SimulatorOn = False
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
  rtde_help.setTCPoffset([0, 0, 0.035, 0, 0, 0])
  rospy.sleep(0.2)

  if FT_SimulatorOn:
    print("wait for FT simul")
    rospy.wait_for_service('start_sim')
    # bring the service
    netftSimCall = rospy.ServiceProxy('start_sim', StartSim)


  # Set the synchronization Publisher
  syncPub = rospy.Publisher('sync', Int8, queue_size=1)
  syncPub.publish(SYNC_RESET)

  print("Wait for the data_logger to be enabled")
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
  dataLoggerEnable(False) # reset Data Logger just in case
  rospy.sleep(1)
  file_help.clearTmpFolder()        # clear the temporary folder
  datadir = file_help.ResultSavingDirectory
  
  # Set the disengage pose
  disengagePosition = [-0.516, .303, 0.0643]
  setOrientation = tf.transformations.quaternion_from_euler(pi,0,pi/2 + pi/180*(-30),'sxyz') #static (s) rotating (r)
  disEngagePose = rtde_help.getPoseObj(disengagePosition, setOrientation)
  
  
  # set initial parameters
  suctionSuccessFlag = False
  

  # try block so that we can have a keyboard exception
  try:

    input("Press <Enter> to go to start pose")
    startPose = copy.deepcopy(disEngagePose)
    startPose.pose.position.z = disEngagePose.pose.position.z + 0.002
    rtde_help.goToPose(startPose)
    rospy.sleep(1)
    # set biases now
    try:
      FT_help.setNowAsBias()
      rospy.sleep(0.1)
    except:
      print("set now as offset failed, but it's okay")

   
    input("Start the cyclic loading test")
    # loop for the number of cyclic loading
    for i in range(args.startidx, args.cycle+1):

      rtde_help.goToPose(disEngagePose)
      rospy.sleep(0.8)
      # check the pose difference
      measuredCurrPose = rtde_help.getCurrentPose()
      pose_diff = measuredCurrPose.pose.position.z - disEngagePose.pose.position.z # need to compensate the difference between the desired pose and the actual pose
      args.pose_diff_disengage = pose_diff
      print("pose_diff_disengage: ", pose_diff)
      dataLoggerEnable(True)
      rospy.sleep(0.5)


      # go to the target pose
      targetPose = copy.deepcopy(disEngagePose)
      targetPose.pose.position.z = disEngagePose.pose.position.z - 0.002
      rtde_help.goToPose(targetPose)
      rospy.sleep(0.8)
      # measure the force
      F_normal = FT_help.averageFz_noOffset
      args.F_normal = -F_normal
      args.iteration_count = i
      # check the pose difference
      measuredCurrPose = rtde_help.getCurrentPose()
      pose_diff = measuredCurrPose.pose.position.z - targetPose.pose.position.z # need to compensate the difference between the desired pose and the actual pose
      print("pose_diff_target: ", pose_diff)
      args.pose_diff_target = pose_diff


      # stop data logging
      rospy.sleep(0.2)
      dataLoggerEnable(False)
      rospy.sleep(0.2)

      # save data and clear the temporary folder
      file_help.saveDataParams(args, appendTxt='JP_HRCC_cyclic_load_2mm_' + str(args.iteration_count)+'_foce_'+ str(args.F_normal))
      file_help.clearTmpFolder()


    # go to disengage pose
    print("Start to go to start pose")
    rtde_help.goToPose(startPose)

    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--startidx', type=int, help='start index of the cyle', default= 1)
  parser.add_argument('--cycle', type=int, help='the number of cycle', default= 10000)

  args = parser.parse_args()    
  main(args)
