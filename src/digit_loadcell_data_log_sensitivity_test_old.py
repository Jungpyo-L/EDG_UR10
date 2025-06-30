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
import geometry_msgs.msg

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


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
  file_help = fileSaveHelp(saveFrames=False)
  rospy.sleep(0.5)
  rtde_help = rtdeHelp(125)
  base_z_speed = 2e-6
  adpt_help = adaptMotionHelp(d_w = 1,d_lat = 10e-3, d_z= base_z_speed) # need to change d_z to change the speed of the robot
  rospy.sleep(0.5)

  # Set up DIGIT frame subscriber
  bridge = CvBridge()
  digit_frames = []
  record_digit = False

  def digit_callback(msg):
    if record_digit:
        try:
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            digit_frames.append(frame)
        except Exception as e:
            print(f"Failed to convert image: {e}")

  rospy.Subscriber("digitFrame", Image, digit_callback)


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
  positionA = [0.604, -0.170, 0.022]   # for test sheet on 3 metal plates
  # positionA = [0.600, -0.140, 0.021]   # 0.04 for 3 metal plates and texture cube
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
    # start data logging with video recording
    record_digit = True
    digit_frames.clear()
    dataLoggerEnable(True)
    rospy.sleep(0.2)

    # initial setup
    F_normal = FT_help.averageFz_noOffset
    targetPoseEngaged = rtde_help.getCurrentPose()
    targetPose = targetPoseEngaged  # Initialize targetPose
    syncPub.publish(SYNC_START)

    # set movement parameters
    step_time = 1
    T_down = adpt_help.get_Tmat_TranslateInZ(direction = 1)
    T_up = adpt_help.get_Tmat_TranslateInZ(direction = -1)

    print("Controls: w = up, s = down, e = exit")

    # force setpoint
    force_setpoint = 0

    while True:
      key = input('Press a key (w/s/e): ').strip().lower()

      # update targetPoseEngaged and F_normal every iteration
      targetPoseEngaged = rtde_help.getCurrentPose()
      targetPose = targetPoseEngaged
      F_normal = FT_help.averageFz_noOffset

      force_inc = 1

      # deal with inputs
      if key == 'w':
        force_setpoint += force_inc
      elif key == 's':
        force_setpoint -= force_inc
      elif key == 'e':
        print("Exiting the loop and returning to pose A.")
        rtde_help.stopAtCurrPoseAdaptive()
        syncPub.publish(SYNC_STOP)
        break

      print(f"fz_setpoint: {force_setpoint:.4f}, fz: {F_normal:.4f}")

      # now, let's use two-step proportional controller to get to force setpoint
      if F_normal > force_setpoint:
        while (F_normal - force_setpoint) > 0.1 and F_normal > -30:
          # set speed proportional to force error
          speed_multiplier = min(1, abs(F_normal - force_setpoint) / 20.0)  # adjust the divisor for sensitivity
          adpt_help.d_z_normal = max(base_z_speed/40, base_z_speed * speed_multiplier)  # ensure speed is not too low
          
          # move
          targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_down, targetPose)
          rtde_help.goToPoseAdaptive(targetPose, time=step_time)

          targetPoseEngaged = rtde_help.getCurrentPose()
          F_normal = FT_help.averageFz_noOffset

          print("down")

        rospy.sleep(0.5)

        # approach setpoint in other direction now
        while (force_setpoint - F_normal) > 0.2 and F_normal > -30:
          adpt_help.d_z_normal = 1e-7
          targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_up, targetPose)
          rtde_help.goToPoseAdaptive(targetPose, time=step_time)

          targetPoseEngaged = rtde_help.getCurrentPose()
          F_normal = FT_help.averageFz_noOffset
          print('adjusting')

      
      elif F_normal < force_setpoint:
        while (force_setpoint - F_normal) > 0.1 and F_normal > -30 and targetPoseEngaged.pose.position.z < positionA[2]:
          # set speed proportional to force error
          speed_multiplier = min(1, abs(F_normal - force_setpoint) / 20.0)  # adjust the divisor for sensitivity
          adpt_help.d_z_normal = max(base_z_speed/100, base_z_speed * speed_multiplier)  # ensure speed is not too low
          
          # move
          targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_up, targetPose)
          rtde_help.goToPoseAdaptive(targetPose, time=step_time)

          targetPoseEngaged = rtde_help.getCurrentPose()
          F_normal = FT_help.averageFz_noOffset

          print("up")

        rospy.sleep(0.5)

        # approach setpoint in other direction now
        while (F_normal - force_setpoint) > 0.2 and F_normal > -30:
          adpt_help.d_z_normal = 1e-7
          targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_down, targetPose)
          rtde_help.goToPoseAdaptive(targetPose, time=step_time)

          targetPoseEngaged = rtde_help.getCurrentPose()
          F_normal = FT_help.averageFz_noOffset
          print('adjusting')

      # default to no motion
      rtde_help.stopAtCurrPoseAdaptive()

      # wait a little
      rospy.sleep(1)

      # print current pose and force, limit to 4 decimal places
      print(f"z: {targetPoseEngaged.pose.position.z:.4f}, Fz: {F_normal:.4f}")

    # after loop ends, stop motion, return to original pose
    # stop data logging
    record_digit = False
    dataLoggerEnable(False)
    rospy.sleep(0.5)
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
  
# functions for keyboard control
def on_press(key):
  global moving_up, moving_down, exit_requested
  try:
    if key -- keyboard.Key.up:
      moving_up = True
    elif key == keyboard.Key.down:
      moving_down = True
    elif key.char == 'e':
      exit_requested = True
  except AttributeError:
    pass
def on_release(key):
  global moving_up, moving_down
  try:
    if key == keyboard.Key.up:
      moving_up = False
    elif key == keyboard.Key.down:
      moving_down = False
  except AttributeError:
    pass

if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--int', type=int, help='argument for int type', default= 100)
  parser.add_argument('--str', type=str, help='argument for str type', default= "string")
  parser.add_argument('--bool', type=bool, help='argument for bool type', default= True)

  args = parser.parse_args()    
  main(args)