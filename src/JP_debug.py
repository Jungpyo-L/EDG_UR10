#!/usr/bin/env python
# Created by Nimran Shergill (Jan 27th 2025) Adapted from Shergill_SM24_2.py which is also adapted from Jungpyo Lee's adaptive motion codes
# This is a file for pure horizontal motion with several rotations. Goal is to capture lift and drag changes as a function 
# of angle but also to capture to linear skin drag effect.

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
from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp

# def significant_motion_check(currentPose, targetPose, position_threshold=1e-3, rotation_threshold=1e-2):
#     position_diff = np.linalg.norm(
#         np.array([currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z]) -
#         np.array([targetPose.pose.position.x, targetPose.pose.position.y, targetPose.pose.position.z])
#     )
#     rotation_diff = np.linalg.norm(
#         np.array([currentPose.pose.orientation.x, currentPose.pose.orientation.y,
#                   currentPose.pose.orientation.z, currentPose.pose.orientation.w]) -
#         np.array([targetPose.pose.orientation.x, targetPose.pose.orientation.y,
#                   targetPose.pose.orientation.z, targetPose.pose.orientation.w])
#     )
#     return position_diff > position_threshold or rotation_diff > rotation_threshold

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
  #################################################
  ## Rise out of media. No rotation.
  currentPose = rtde_help.getCurrentPose()
  PositionA = [currentPose.pose.position.x, currentPose.pose.position.y, 0.4]
  OrientationA = [currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w]
  PoseA = rtde_help.getPoseObj(PositionA, OrientationA)

  # Pose B has to be defined relative to A so it is defined during the motion sequence

  # We descend into media. No rotation. 
  PositionC = [0.320, -0.200, 0.12]
  OrientationC = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi,'sxyz') # not moving it from the previous transformation
  PoseC = rtde_help.getPoseObj(PositionC, OrientationC) 
  ##################################################
                    #############
  ##################################################

################ REACH STARTING POSITION ########################
  try:
    # POSE A
    input("Press <Enter> to go to PoseA")
    rtde_help.goToPose(PoseA) 
    rospy.sleep(1)

    # POSE B
    input("Press <Enter> to go to PoseB")
    currentPose = rtde_help.getCurrentPose()
    PositionB = [0.320, -0.200, currentPose.pose.position.z] # change the first two parameters to be the "beginning of the tank"
    OrientationB = tf.transformations.quaternion_from_euler(np.pi, 0,-np.pi,'sxyz') #static (s) rotating (r)
    #   Note the new coordinates: x is pointing to us, y is pointing to the left, and z is pointing down.
    PoseB = rtde_help.getPoseObj(PositionB, OrientationB)
    rtde_help.goToPose(PoseB) 
    rospy.sleep(1)

    # POSE C
    input("Press <Enter> to go to PoseC")
    rtde_help.goToPose(PoseC) 
    rospy.sleep(1)

    # ZERO GRAVITY AND OTHER FORCES
    FT_help.setNowAsBias() # offset the force sensor, zeros gravity and other forces
    args.ForceOffset1 = [FT_help.offSetFx, FT_help.offSetFy, FT_help.offSetFz, FT_help.offSetTx, FT_help.offSetTy, FT_help.offSetTz]

# Start horizontal motion with rotations
    input("Press <Enter> to snout motion sequence with horizontal motion + rotations")
    dataLoggerEnable(True)

    # Initializing variables to retain global frame
    # THese are the variables I don't remember if I use
    T_start = adpt_help.get_Tmat_from_Pose(currentPose)
    R_start = T_start[:3, :3]
    
    # This is the horizontal world frame motion translation matrix
    T_horiz_world = adpt_help.get_Tmat_TranlateInX(direction=-1)
    tvec_horiz_world = np.array([-0.01, 0, 0]) # move in the negative x direction in the world frame
    T_horiz_rotated = np.eye(4)

    overall_angle = 0 # Initialize cumulative rotation angle

################ START MOTION #####################################
  ##################################################
  #                  # HORIZ 1 #                   #
  #                  # edit line 156               #
  ##################################################
    syncPub.publish(1)
    currentPose = rtde_help.getCurrentPose()
    current_x = currentPose.pose.position.x
    while currentPose.pose.position.x <= current_x + 0.05:
      currentPose = rtde_help.getCurrentPose()
      targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_horiz_world, currentPose)
      rtde_help.goToPoseAdaptive(targetPose, time=0.05)
    rtde_help.stopAtCurrPoseAdaptive()
  ##################################################
  #                   # ROT 1 #                    #
  #                   # edit lines 166, 184        #
  ##################################################
    adpt_help.dw = 0.01
    syncPub.publish(2)
    while overall_angle <= 12.5:
        T_rot_step = adpt_help.get_Tmat_RotateInY(direction=1) # Positive Y-direction  
        currentPose = rtde_help.getCurrentPose()
        targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
        rtde_help.goToPoseAdaptive(targetPose, time=0.05)

        # Updating the overall angle
        currentPose = rtde_help.getCurrentPose()
        T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
        T_overall = np.linalg.inv(T_start) @ T_curr # local frame to the world frame
        # Angle relative to the start
        overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi 
        
        # Not needed here since we know the direction of rotation and want a final positive angle, will be useful in the future
        # if T_overall[2, 0] > 0:  # Check direction of rotation based on off-diagonal terms
        #   overall_angle = -overall_angle

        # Condition to break the loop
        # if overall_angle >= 12.5:
        #     rtde_help.stopAtCurrPoseAdaptive()
        #     break
    rtde_help.stopAtCurrPoseAdaptive()
    
  # ##################################################
  # #                  # HORIZ 2 #                   #
  # #                  # edit lines 207              #
  # ##################################################
    FT_help.setNowAsBias() # offset the force sensor, zeros gravity and other forces
    R_relative = T_overall[:3,:3] 
    # Dealing with the horizontal motion in the local frame after rotation
    t_horiz_local = R_relative.T @ tvec_horiz_world # translation VECTOR for horizontal motion
    T_horiz_rotated[:3,3] = t_horiz_local

    syncPub.publish(3) # giving a gap between rotation and next motion to lessen oscillations
    rospy.sleep(1.5)
   # FT_help.setNowAsBias()
    args.ForceOffset2 = [FT_help.offSetFx, FT_help.offSetFy, FT_help.offSetFz, FT_help.offSetTx, FT_help.offSetTy, FT_help.offSetTz] 
    syncPub.publish(4)

    currentPose = rtde_help.getCurrentPose()
    current_x = currentPose.pose.position.x
    while currentPose.pose.position.x <= current_x + 0.05:
      currentPose = rtde_help.getCurrentPose()
      targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_horiz_rotated, currentPose)
      rtde_help.goToPoseAdaptive(targetPose, time=0.05)
    rtde_help.stopAtCurrPoseAdaptive()

  # ##################################################
  # #                  # ROTATION 2 #                #
  # #                   # edit lines 216, 235        #
  # ##################################################
  #   syncPub.publish(5)
  #   while overall_angle >= -12.5: # negative rotation
  #       adpt_help.dw = 0.01
  #       T_rot_step = adpt_help.get_Tmat_RotateInY(direction=-1) # Positive Y-direction  
  #       currentPose = rtde_help.getCurrentPose()
  #       targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
  #       rtde_help.goToPoseAdaptive(targetPose, time=0.05)
        
  #       # Updating the overall angle
  #       currentPose = rtde_help.getCurrentPose()
  #       T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
  #       T_overall = np.linalg.inv(T_start) @ T_curr # local frame to the world frame
  #       # Angle relative to the start
  #       overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi 
        
  #       # Needed here because of crossover
  #       if T_overall[2, 0] > 0:  # Check direction of rotation based on off-diagonal terms
  #           overall_angle = -overall_angle

  #       # Condition to break the loop
  #       if overall_angle <= -12.5:
  #           rtde_help.stopAtCurrPoseAdaptive()
  #           break

  # ##################################################
  # #                  # HORIZ 3 #                   #
  # #                  # edit lines 256              #
  # ##################################################
  #   FT_help.setNowAsBias() # offset the force sensor, zeros gravity and other forces
  #   R_relative = T_overall[:3,:3] 
  #   # Dealing with the horizontal motion in the local frame after rotation
  #   t_horiz_local = R_relative.T @ tvec_horiz_world # translation VECTOR for horizontal motion
  #   T_horiz_rotated[:3,3] = t_horiz_local

  #   syncPub.publish(6) # giving a gap between rotation and next motion to lessen oscillations
  #   rospy.sleep(1.5)
  #  # FT_help.setNowAsBias()
  #   args.ForceOffset2 = [FT_help.offSetFx, FT_help.offSetFy, FT_help.offSetFz, FT_help.offSetTx, FT_help.offSetTy, FT_help.offSetTz] 
  #   syncPub.publish(7)

  #   currentPose = rtde_help.getCurrentPose()
  #   current_x = currentPose.pose.position.x
  #   while currentPose.pose.position.x <= current_x + 0.05:
  #     currentPose = rtde_help.getCurrentPose()
  #     targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_horiz_rotated, currentPose)
  #     rtde_help.goToPoseAdaptive(targetPose, time=0.05)
  #  ##################################################
  #  #                 # ROTATION 3 #                 #
  #  #                 # edit lines 265, 267 284      #
  # ################################################### 
  #   syncPub.publish(8)
  #   while overall_angle <= 0: # positive rotation
  #       adpt_help.dw = 0.01
  #       T_rot_step = adpt_help.get_Tmat_RotateInY(direction=1) # Positive Y-direction  
  #       currentPose = rtde_help.getCurrentPose()
  #       targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
  #       rtde_help.goToPoseAdaptive(targetPose, time=0.05)
        
  #       # Updating the overall angle
  #       currentPose = rtde_help.getCurrentPose()
  #       T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
  #       T_overall = np.linalg.inv(T_start) @ T_curr # local frame to the world frame
  #       # Angle relative to the start
  #       overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi 
        
  #       # Needed here because of crossover
  #       if T_overall[2, 0] > 0:  # Check direction of rotation based on off-diagonal terms
  #           overall_angle = -overall_angle

  #       # Condition to break the loop
  #       if overall_angle >= 0:
  #           rtde_help.stopAtCurrPoseAdaptive()
  #           break
  # ##################################################
  # #                  # HORIZ 4 #                   #
  # #                   # edit lines 304             #
  # ##################################################
  #   FT_help.setNowAsBias() # offset the force sensor, zeros gravity and other forces
  #   R_relative = T_overall[:3,:3] 
  #   # Dealing with the horizontal motion in the local frame after rotation
  #   t_horiz_local = R_relative.T @ tvec_horiz_world # translation VECTOR for horizontal motion
  #   T_horiz_rotated[:3,3] = t_horiz_local

  #   syncPub.publish(9) # giving a gap between rotation and next motion to lessen oscillations
  #   rospy.sleep(1.5)
  #   # FT_help.setNowAsBias()
  #   args.ForceOffset2 = [FT_help.offSetFx, FT_help.offSetFy, FT_help.offSetFz, FT_help.offSetTx, FT_help.offSetTy, FT_help.offSetTz] 
  #   syncPub.publish(10)

  #   currentPose = rtde_help.getCurrentPose()
  #   current_x = currentPose.pose.position.x
  #   while currentPose.pose.position.x <= current_x + 0.05:
  #     currentPose = rtde_help.getCurrentPose()
  #     targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_horiz_rotated, currentPose)
  #     rtde_help.goToPoseAdaptive(targetPose, time=0.05)
  # ##################################################
  # #                 # ROTATION 4 #                 #
  # #                # edit lines 265, 284           #
  # ################################################## 
  #   syncPub.publish(11)
  #   while overall_angle <= 20: # negative rotation
  #       adpt_help.dw = 0.01
  #       T_rot_step = adpt_help.get_Tmat_RotateInY(direction= 1) # Positive Y-direction  
  #       currentPose = rtde_help.getCurrentPose()
  #       targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
  #       rtde_help.goToPoseAdaptive(targetPose, time=0.05)
        
  #       # Updating the overall angle
  #       currentPose = rtde_help.getCurrentPose()
  #       T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
  #       T_overall = np.linalg.inv(T_start) @ T_curr # local frame to the world frame
  #       # Angle relative to the start
  #       overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi 
        
  #       # Needed here because of crossover
  #       if T_overall[2, 0] > 0:  # Check direction of rotation based on off-diagonal terms
  #           overall_angle = -overall_angle

  #       # Condition to break the loop
  #       if overall_angle >= 20:
  #           rtde_help.stopAtCurrPoseAdaptive()
  #           break
  # ##################################################
  # #                  # HORIZ 5 #                   #
  # #                   # edit lines 351             #
  # ##################################################
  #   FT_help.setNowAsBias() # offset the force sensor, zeros gravity and other forces
  #   R_relative = T_overall[:3,:3] 
  #   # Dealing with the horizontal motion in the local frame after rotation
  #   t_horiz_local = R_relative.T @ tvec_horiz_world # translation VECTOR for horizontal motion
  #   T_horiz_rotated[:3,3] = t_horiz_local

  #   syncPub.publish(12) # giving a gap between rotation and next motion to lessen oscillations
  #   rospy.sleep(1.5)
  #   # FT_help.setNowAsBias()
  #   args.ForceOffset2 = [FT_help.offSetFx, FT_help.offSetFy, FT_help.offSetFz, FT_help.offSetTx, FT_help.offSetTy, FT_help.offSetTz] 
  #   syncPub.publish(13)

  #   currentPose = rtde_help.getCurrentPose()
  #   current_x = currentPose.pose.position.x
  #   while currentPose.pose.position.x <= current_x + 0.05:
  #     currentPose = rtde_help.getCurrentPose()
  #     targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_horiz_rotated, currentPose)
  #     rtde_help.goToPoseAdaptive(targetPose, time=0.05)
  #   ################ CONCLUDING MOTION #####################################      
  #   dataLoggerEnable(False)
  #   # save data and clear the temporary folder
  #   file_help.saveDataParams(args, appendTxt='Shergill_HORIZONTAL_Snout_Experiment'+'beta_'+str(args.beta)+ '_trial_'+str(args.trialNum))
  #   file_help.clearTmpFolder()        # clear the temporary folder

  #   # Print the final orientation
  #   currentPose = rtde_help.getCurrentPose()
  #   # Quaternion components
  #   qx = currentPose.pose.orientation.x
  #   qy = currentPose.pose.orientation.y
  #   qz = currentPose.pose.orientation.z
  #   qw = currentPose.pose.orientation.w

  #   # Convert quaternion to rotation matrix
  #   rotation_matrix = np.array([
  #       [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
  #       [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
  #       [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)]
  #   ])

  #   # Extract roll, pitch, and yaw
  #   # Roll (x-axis rotation)
  #   roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])

  #   # Pitch (y-axis rotation)
  #   # Clamping to avoid numerical instability
  #   pitch = np.arcsin(-rotation_matrix[2, 0])

  #   # Yaw (z-axis rotation)
  #   yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

  #   # Convert to 
  #   roll_deg = np.degrees(roll)
  #   pitch_deg = np.degrees(pitch)
  #   yaw_deg = np.degrees(yaw)

    # print(f"Roll (x-axis): {roll_deg:.2f} degrees")
    # print(f"Pitch (y-axis): {pitch_deg:.2f} degrees")
    # print(f"Yaw (z-axis): {yaw_deg:.2f} degrees")
    # print("============ Python UR_Interface demo complete!")

     #   ################ CONCLUDING MOTION #####################################      
    dataLoggerEnable(False)
    # save data and clear the temporary folder
    file_help.saveDataParams(args, appendTxt='debugging')
    file_help.clearTmpFolder()        # clear the temporary folder

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  

if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--beta', type=float, help='beta angle of wedge', default= 25) # wrote zero so I realize I didn't change this parameter
  parser.add_argument('--rotationAngle1', type=float, help='Rotation angle 1', default= 12.5)
  parser.add_argument('--rotationAngle2', type=float, help='Rotation angle 2', default= -12.5)
  parser.add_argument('--rotationAngle3', type=float, help='Rotation angle 3', default= 0)  
  parser.add_argument('--rotationAngle4', type=float, help='Rotation angle 4', default= 20) 
  parser.add_argument('--trialNum', type=int, help='Trial number', default= 1)
  args = parser.parse_args()    

  main(args)


