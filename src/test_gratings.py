#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: July.15.2025
# Last update: July.15.2025
# Description: records digit sensor data unloaded and loaded on different gratings
# Version: 0.1 (July.15.2025) - creation

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
  adpt_help = adaptMotionHelp(d_w = 1,d_lat = 10e-3, d_z = test_config.GRATINGS_Z_SPEED) # need to change d_z to change the speed of the robot
  rospy.sleep(0.5)
  rtde_help.setTCPoffset(test_config.VBTS_TCP_OFFSET)

  # Set the synchronization Publisher
  syncPub = rospy.Publisher('sync', Int8, queue_size=1)

  print("Wait for the data_logger to be enabled")
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
  dataLoggerEnable(False) # reset Data Logger just in case
  print("Wait for digit frame toggle service")
  rospy.wait_for_service('capture_digit_frame')
  capture_digit = rospy.ServiceProxy('capture_digit_frame', SetBool)
  rospy.sleep(1)
  file_help.clearTmpFolder()        # clear the temporary folder
  datadir = file_help.ResultSavingDirectory


  # Set the pose A
  positionA = test_config.GRATINGS_POS_A   # for gratings
  orientationA = tf.transformations.quaternion_from_euler(np.pi+0.012,0.012,-np.pi/2,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA, orientationA)

  # pose B is loaded pose
  poseB = poseA
  poseB_found = False
  

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
    print("Recording noload data...")
    # start data logging with video recording
    dataLoggerEnable(True)
    save_frames(capture_digit)
    syncPub.publish(SYNC_START)
    rospy.sleep(test_config.SAVE_PERIOD)
    rospy.sleep(0.2)

    print("loading...")

    # flags and variables
    
    farFlag = True
    # slow approach until it reach target height
    F_normal = FT_help.averageFz_noOffset
    targetPoseEngaged = rtde_help.getCurrentPose()
    targetPose = targetPoseEngaged  # Initialize targetPose
    # targetPWM_Pub.publish(DUTYCYCLE_0)
    while farFlag:
      # load
      while targetPoseEngaged.pose.position.z > 0.00 and F_normal > -test_config.GRATINGS_FORCE_THRESHOLD:
        T_move = adpt_help.get_Tmat_TranslateInZ(direction = 1)
        targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, targetPose)
        rtde_help.goToPoseAdaptive(targetPose, time = 0.1)

        # new z height
        targetPoseEngaged = rtde_help.getCurrentPose()
        F_normal = FT_help.averageFz_noOffset

        if not poseB_found and F_normal < -9.9:
          poseB = targetPoseEngaged
          poseB_found = True

        # print(F_normal, FT_help.thisForce.force.z)

      print("current normal force: ", F_normal)
      farFlag = False
      rtde_help.stopAtCurrPoseAdaptive()
      targetPose = rtde_help.getCurrentPose()  # Update targetPose after stopping
      rospy.sleep(1)
      print("reached threshhold normal force: ", F_normal)
      args.normalForceUsed= F_normal
      rospy.sleep(0.8)

    # go to pose B
    # print("Going to pose B")
    # rtde_help.goToPose(poseB)
    # rospy.sleep(1)

    # now log 1 second of loaded data
    print("Recording loaded data...")
    save_frames(capture_digit)
    syncPub.publish(SYNC_STOP)
    dataLoggerEnable(False)
    rospy.sleep(.5)
        
    # back to pose A
    rtde_help.goToPose(poseA)
    rospy.sleep(.1)


    # # save data and clear the temporary folder
    # file_help.saveDataParams(args, appendTxt='digit_data_log_'+'Test', image_frames=digit_frames)
    # file_help.clearTmpFolder()

    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  

# function for saving certain number of frames
def save_frames(capture_digit, wait_time=test_config.SAVE_PERIOD):
  capture_digit(True)
  rospy.sleep(wait_time)  # Wait for the specified save period


if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--int', type=int, help='argument for int type', default= 100)
  parser.add_argument('--str', type=str, help='argument for str type', default= "string")
  parser.add_argument('--bool', type=bool, help='argument for bool type', default= True)

  args = parser.parse_args()    
  main(args)