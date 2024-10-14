#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: Oct.14.2024
# Last update: Oct.14.2024
# Description: This script is primarily for basic experiment demonstration for UR10e robot while logging the data.
# It has two different tests 
# 1. move Pose A to Pose B, then collect data for 10 s and back to Pose A
# 2. move Pose A to Pose B, then rotate by 45 degrees in y-axis, then collect data for 10 s and back to Pose A
# With argument --test 0, it will execute the first test, and --test 1 will execute the second test.
# With argument --cycle, you can specify the number of cycle to apply.

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

  SYNC_RESET = 0
  SYNC_START = 1
  SYNC_STOP = 2
  
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

  print("Wait for the data_logger to be enabled")
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
  dataLoggerEnable(False) # reset Data Logger just in case
  rospy.sleep(1)
  file_help.clearTmpFolder()        # clear the temporary folder
  datadir = file_help.ResultSavingDirectory


  # Set the pose A
  positionA = [0.520, -0.200, 0.20]
  orientationA = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA, orientationA)

  # Set the pose B
  positionB = [0.620, -0.100, 0.30]
  orientationA = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseB = rtde_help.getPoseObj(positionB, orientationA)

  # Set the pose C
  positionB = [0.620, -0.100, 0.30]
  orientationB  = tf.transformations.quaternion_from_euler(np.pi + 45*deg2rad,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseC = rtde_help.getPoseObj(positionB, orientationB)
  

  # try block so that we can have a keyboard exception
  try:

    input("Press <Enter> to go t start expeirment")
    # set biases now
    try:
      FT_help.setNowAsBias()
      rospy.sleep(0.1)
    except:
      print("set now as offset failed, but it's okay")

    # start data logging
    dataLoggerEnable(True)
    rospy.sleep(0.2)
    
    # for loop for the number of cycle
    for i in range(args.cycle):
      # move to Pose A
      rtde_help.goToPose(poseA)
      rospy.sleep(0.2)

      # move to Pose B
      rtde_help.goToPose(poseB)
      rospy.sleep(0.2)
      syncPub.publish(SYNC_START)

      # while loop for 10 s
      start_time = time.time()
      while (time.time() - start_time) < 10:        
        rospy.sleep(0.1)

      syncPub.publish(SYNC_STOP)

      # move to Pose A
      rtde_help.goToPose(poseA)
      rospy.sleep(0.2)
      print("poseA: ", rtde_help.getCurrentPose())

   
    input("Press <Enter> to start to record data")
    

    # while loop for 10 s
    start_time = time.time()
    while (time.time() - start_time) < 10:
      # publish SYNC_START
      syncPub.publish(SYNC_START)
      rospy.sleep(0.1)


    # stop data logging
    dataLoggerEnable(False)
    rospy.sleep(0.2)

    # save data and clear the temporary folder
    file_help.saveDataParams(args, appendTxt='Simple_experiment_'+'author'+str(args.author)+'testNumber_'+ str(args.test)+'_cycle_'+ str(args.cycle))
    file_help.clearTmpFolder()

    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--test', type=int, help='argument for test type', default= 0)
  parser.add_argument('--author', type=str, help='argument for str type', default= "EDG")
  parser.add_argument('--cycle', type=int, help='the number of cycle to apply', default = 1)

  args = parser.parse_args()    
  main(args)
