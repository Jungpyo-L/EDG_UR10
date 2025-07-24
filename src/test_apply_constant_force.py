#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: May.05.2025
# Last update: May.08.2025
# Description: This script is primarily for basic data logging using UR10e robot, ATI sensor, and digit tactile sensor. 
# It record ATI sensor data and digit frame data while it moves the robot in z direction.
# It is a simple example of how to use the data logger and how to record data.
# Version: 0.1
# Date: May.05.2025
# This script is for the UR10e robot with ATI sensor and digit tactile sensor.
# Version: 0.2
# Date: May.08.2025
# This script is updated to include robot motion (move to A, and adaptive motion, and back to A)

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
from std_srvs.srv import SetBool
import geometry_msgs.msg

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import test_config


current_dir = os.path.dirname(os.path.abspath(__file__))
helper_path = os.path.join(current_dir, "helperFunction")
sys.path.append(helper_path)

from FT_callback_helper import FT_CallbackHelp
from fileSaveHelper import fileSaveHelp
from rtde_helper import rtdeHelp
from adaptiveMotion import adaptMotionHelp


def main(args):

  SYNC_RESET = 0
  SYNC_START = 1
  SYNC_STOP = 2

  np.set_printoptions(precision=4)

  # controller node
  rospy.init_node('edg_experiment')

  # Setup helper functions
  FT_help = FT_CallbackHelp() # it deals with subscription.
  rospy.sleep(0.5)
  file_help = fileSaveHelp(saveFrames=True)
  rospy.sleep(0.5)
  rtde_help = rtdeHelp(125)
  z_speed = 1e-6
  adpt_help = adaptMotionHelp(d_w = 1,d_lat = 10e-3, d_z = test_config.SENS_Z_SPEED) # need to change d_z to change the speed of the robot
  rospy.sleep(0.5)
  rtde_help.setTCPoffset(test_config.VBTS_TCP_OFFSET)


  # Set the synchronization Publisher
  syncPub = rospy.Publisher('sync', Int8, queue_size=1)

  print("Wait for the data_logger to be enabled")
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
  dataLoggerEnable(False) # reset Data Logger just in case
  print("Wait for digit frame toggle service")
  rospy.wait_for_service('toggle_digit_frame')
  toggle_digit = rospy.ServiceProxy('toggle_digit_frame', SetBool)
  rospy.sleep(1)
  file_help.clearTmpFolder()        # clear the temporary folder
  datadir = file_help.ResultSavingDirectory


  # Set the pose A
  # positionA = test_config.SENS_POS_A   # for sensitivity board
  positionA = test_config.ABRASION_POS_A   # for abrasion test
  orientationA = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA, orientationA)
  

  # try block so that we can have a keyboard exception
  try:

    input("Press <Enter> to go to set bias")
    # set biases now
    try:
      FT_help.setNowAsBias()
      rospy.sleep(0.1)
    except:
      print("set now as offset failed, but it's okay")

    input("Press <Enter> to go to pose A")
    rtde_help.goToPose(poseA)
    rospy.sleep(1)
   
    input("Press <Enter> to start to record data")
    # start data logging with video recording. record 1 second of noload data
    record_digit = True
    dataLoggerEnable(True)
    toggle_digit(True)
    syncPub.publish(SYNC_START)
    rospy.sleep(0.2)

    # flags and variables
    
    farFlag = True
    # slow approach until it reach target height
    F_normal = FT_help.averageFz_noOffset
    targetPoseEngaged = rtde_help.getCurrentPose()
    targetPose = targetPoseEngaged  # Initialize targetPose
    # targetPWM_Pub.publish(DUTYCYCLE_0)
    while farFlag:
        # load
        while targetPoseEngaged.pose.position.z > 0.00 and F_normal > -test_config.CONST_FORCE_THRESHOLD:
          T_move = adpt_help.get_Tmat_TranslateInZ(direction = 1)
          targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, targetPose)
          rtde_help.goToPoseAdaptive(targetPose, time = 0.1)

          # new z height
          targetPoseEngaged = rtde_help.getCurrentPose()
          F_normal = FT_help.averageFz_noOffset

          # print(F_normal, FT_help.thisForce.force.z)

        # stop robot
        rtde_help.stopAtCurrPoseAdaptive()
        rospy.sleep(0.5)

        F_normal = FT_help.averageFz_noOffset

        # record 1 second of data
        # print(f'Recording loaded data at {F_normal} N...')

        # change motor speed
        adpt_help.d_z_normal = 2e-5

        input('done recording, press <Enter> to unload')

        # unload
        while targetPoseEngaged.pose.position.z < positionA[2]:
          T_move = adpt_help.get_Tmat_TranslateInZ(direction = -1)
          targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, targetPose)
          rtde_help.goToPoseAdaptive(targetPose, time = 0.1)

          # new z height
          targetPoseEngaged = rtde_help.getCurrentPose()
          F_normal = FT_help.averageFz_noOffset

          # print(F_normal, FT_help.thisForce.force.z)

        farFlag = False
        rtde_help.stopAtCurrPoseAdaptive()
        print("reached threshhold normal force: ", F_normal)
        args.normalForceUsed= F_normal
        rospy.sleep(0.8)

    # stop data logging
    rospy.sleep(1)
    print(f'Recording final data')
    toggle_digit(False)
    syncPub.publish(SYNC_STOP)
    dataLoggerEnable(False)

    # back to pose A
    rtde_help.goToPose(poseA)
    rospy.sleep(1)


    # # save data and clear the temporary folder
    # file_help.saveDataParams(args, appendTxt='digit_data_log_'+'Test', image_frames=digit_frames)
    # file_help.clearTmpFolder()

    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--int', type=int, help='argument for int type', default= 100)
  parser.add_argument('--str', type=str, help='argument for str type', default= "string")
  parser.add_argument('--bool', type=bool, help='argument for bool type', default= True)

  args = parser.parse_args()    
  main(args)