#!/usr/bin/env python

# Authors: Jungpyo Lee, Amber Young
# Create: Oct.08.2024
# Last update: Nov.1.2024
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
  file_help = fileSaveHelp()
  rospy.sleep(0.5)
  rtde_help = rtdeHelp(125, speed=0.1 , acc= 0.1)
  rospy.sleep(0.5)

  # Set the synchronization Publisher
  syncPub = rospy.Publisher('sync', Int8, queue_size=1)

  # Set the TCP offset and calibration matrix (ex, suction cup: 0.150, ATI_default: 0.464)
  # You can set the TCP offset here, but it is recommended to set it in the UR program.
  # If you set it here, endEffectorPose will be different from the actual pose.
  # rtde_help.setTCPoffset([0, 0, 0.464, 0, 0, 0])
  # rospy.sleep(0.2)

  print("Wait for the data_logger to be enabled")
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
  dataLoggerEnable(False) # reset Data Logger just in case
  rospy.sleep(1)
  file_help.clearTmpFolder()        # clear the temporary folder
  datadir = file_help.ResultSavingDirectory

  # Set the pose A
  positionA = [0.536, -.125, .279]
  orientationA = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA, orientationA)

  # Set the pose B
  positionB = [0.536, -.125, .279 - args.depth * 1e-2]
  orientationA = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseB = rtde_help.getPoseObj(positionB, orientationA)

  # Set the pose C
  positionB = [0.536, -.125, .279 - args.depth * 1e-2]
  orientationB  = tf.transformations.quaternion_from_euler(np.pi + 20*deg2rad,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseC = rtde_help.getPoseObj(positionB, orientationB)

  # try block so that we can have a keyboard exception
  try:

    input("Press <Enter> to go start expeirment")
    # set biases now

    # start data logging
    dataLoggerEnable(True)
    rospy.sleep(0.2)

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

    input("Press <Enter> to go to pose B")
    rtde_help.goToPose(poseB)
    rospy.sleep(1)
    print("poseB: ", rtde_help.getCurrentPose())

    
    # stop data logging
    dataLoggerEnable(False)
    rospy.sleep(0.2)

    # save data and clear the temporary folder
    file_help.saveDataParams(args, appendTxt='Simple_experiment_'+'depth_'+str(args.depth)+'_cycle_'+str(args.cycle))
    file_help.clearTmpFolder()

    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--depth', type=int, help='argument for test type', default= 0) # change depth here
  parser.add_argument('--author', type=str, help='argument for str type', default= "EDG")
  parser.add_argument('--cycle', type=int, help='the number of cycle to apply', default = 1)

  args = parser.parse_args()    
  main(args)
