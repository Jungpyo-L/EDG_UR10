#!/usr/bin/env python
## Shergill_SM24_2.py
# Last updated: /2024
# Adapted from Jungpyo's simple_robot_control.py script
# Eventually to add the data collection part but for now just testing the first few motion steps
# This version of code is intended to be a cleaner version of the Shergill_Snout_Motion_2024 code 
# that has more comments in it. 

###########################################################################
# Authors: Nimran Shergill, Jungpyo Lee
# Create: 11.25.2024
# Last update: 11.25.2024
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
import time

from netft_utils.srv import *
from edg_ur10.srv import *


from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.rtde_helper import rtdeHelp
from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.fileSaveHelper import fileSaveHelp


def significant_motion_check(currentPose, targetPose, position_threshold=1e-3, rotation_threshold=1e-2):
    position_diff = np.linalg.norm(
        np.array([currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z]) -
        np.array([targetPose.pose.position.x, targetPose.pose.position.y, targetPose.pose.position.z])
    )
    rotation_diff = np.linalg.norm(
        np.array([currentPose.pose.orientation.x, currentPose.pose.orientation.y,
                  currentPose.pose.orientation.z, currentPose.pose.orientation.w]) -
        np.array([targetPose.pose.orientation.x, targetPose.pose.orientation.y,
                  targetPose.pose.orientation.z, targetPose.pose.orientation.w])
    )
    return position_diff > position_threshold or rotation_diff > rotation_threshold

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

  # d_w is a speed paramter. It is the speed of the rotation in radians per second.
  # d_lat is a speed parameter. It is the speed of the lateral motion in meters per second.
  # d_z is a speed parameter. It is the speed of the vertical motion in meters per second.
  # adpt_help = adaptMotionHelp(d_w = 0.1,d_lat = 10e-3, d_z= 5e-3)

  # Set the TCP offset and calibration matrix (ex, suction cup: 0.150, ATI_default: 0.464)
  # You can set the TCP offset here, but it is recommended to set it in the UR program.
  # If you set it here, endEffect
  # orPose will be different from the actual pose.
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
  #########
  # DEFINE POSES
  #########

  ## Rise out of media. No rotation.
  currentPose = rtde_help.getCurrentPose()
  PositionA = [currentPose.pose.position.x, currentPose.pose.position.y, 0.4]
  OrientationA = [currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w]
  PoseA = rtde_help.getPoseObj(PositionA, OrientationA)

  # We descend into media. No rotation. 
  PositionC = [0.320, -0.200, 0.3]
  OrientationC = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi,'sxyz') # not moving it from the previous transformation
  PoseC = rtde_help.getPoseObj(PositionC, OrientationC) 
  #############################################################################################################################

 # try block so that we can have a keyboard exception
  try:
    # POSE A
    input("Press <Enter> to go to PoseA")
    rtde_help.goToPose(PoseA) 
    currentPose = rtde_help.getCurrentPose()
    rospy.sleep(1)

    # POSE B
    input("Press <Enter> to go to PoseB")
    currentPose = rtde_help.getCurrentPose()
    PositionB = [0.320, -0.200, currentPose.pose.position.z] # change the first two parameters to be the "beginning of the tank"
    OrientationB = tf.transformations.quaternion_from_euler(np.pi, 0,-np.pi,'sxyz') #static (s) rotating (r)
    #   Note the new coordinates: x is pointing to us, y is pointing to the left, and z is pointing down.
    PoseB = rtde_help.getPoseObj(PositionB, OrientationB)
    rtde_help.goToPose(PoseB) 
    currentPose = rtde_help.getCurrentPose()
    rospy.sleep(1)

    # POSE C
    input("Press <Enter> to go to PoseC")
    rtde_help.goToPose(PoseC) 
    rospy.sleep(1)
    currentPose = rtde_help.getCurrentPose()
    
    # ZERO GRAVITY AND OTHER FORCES
    FT_help.setNowAsBias() # offset the force sensor, zeros gravity and other forces

    ########################################
    ########################################
    # ADAPTIVE MOTION
    input("Press <Enter> to snout motion sequence with adaptive control")
    startTime = time.time() 
    currentPose = rtde_help.getCurrentPose()
    T_start = adpt_help.get_Tmat_from_Pose(currentPose) # world frame
    R_start = T_start[:3, :3]  # Rotation matrix of the world frame
    T_horiz_world = adpt_help.get_Tmat_TranlateInX(direction = -1) # move in the negative x direction
    T_vertical_world = adpt_help.get_Tmat_TranlateInZ(direction = 1) # move in the positive z direction

    overall_angle = 0 # Initialize cumulative rotation angle 
    ########################################
    while (time.time() - startTime) < args.timeLimit: # trials for 10 seconds
      ##################################################
      # ADAPTIVE MOTION WHILE LATERAL MOVEMENT HAPPENS #
      ##################################################
      currentPose = rtde_help.getCurrentPose()
      current_x = currentPose.pose.position.x
      while currentPose.pose.position.x <= current_x + 0.05: 
        # # Horizontal motion
        # T_horiz = adpt_help.get_Tmat_TranlateInX(direction = -1) # move in the negative x direction
        # Vertical adaptive motion 
        Fz = FT_help.averageFz_noOffset
        T_normal = adpt_help.get_Tmat_axialMove(Fz, F_normalThres)
        # Combine the motion
        T_move = T_horiz_world @ T_normal
        # Get the target poseR_relative
        currentPose = rtde_help.getCurrentPose()
        targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currentPose)
        # Check if the motion is significant to avoid unnecessary motion
        if significant_motion_check(currentPose, targetPose):
          rtde_help.goToPoseAdaptive(targetPose, time=0.05)

    ##################################################
    #                  ROTATION 1                    #
    ##################################################
    while overall_angle <= 10: # positive rotation
      adpt_help.dw = 0.1
      T_rot_step = adpt_help.get_Tmat_RotateInY(direction=1) # Positive Y-direction  
      currentPose = rtde_help.getCurrentPose()
      targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
      rtde_help.goToPoseAdaptive(targetPose, time=0.05)
      
      # Updating the overall angleR_relative
      currentPose = rtde_help.getCurrentPose()
      T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
      T_overall = np.linalg.inv(T_start) @ T_curr # loca frame to the world frame
      # Angle relative to the start
      overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi 

      # Not needed here since we know the direction of rotation and want a final positive angle, will be useful in the future
      # if T_overall[2, 0] > 0:  # Check direction of rotation based on off-diagonal terms
      #   overall_angle = -overall_angle

      # Condition to break the loop
      if overall_angle >= 10:
        rtde_help.stopAtCurrPoseAdaptive()
        break
    #######################################################################################
    #  ADAPTIVE HORIZONTAL MOTION AFTER ROTATION 1  --> WORLD FRAME MOTION IN LOCAL FRAME #
    #######################################################################################
    FT_help.setNowAsBias() # zero gravity and other forces
    R_relative = T_overall[:3,:3] 
    T_rot = np.eye(4)
    T_rot[:3, :3] = R_relative # embedding the rotation matrix from first rotation into the transformation matrix
    print("R_relative: \n", R_relative)
    currentPose = rtde_help.getCurrentPose()
    current_x = currentPose.pose.position.x
    while currentPose.pose.position.x <= current_x + 0.03:
      # horizontal motion
      T_horiz_local = np.linalg.inv(T_rot) @ T_horiz_world # move in the negative x direction in the local frame
      # vertical adaptive motion
      # Fz = FT_help.averageFz_noOffset
      # T_normal = adpt_help.get_Tmat_axialMove(Fz, F_normalThres)
      # T_vertical_local = np.linalg.inv(T_rot) @ T_vertical_world # move in the positive z direction in the local frame
      # T_combined_vertical = T_vertical_local @ T_normal # combines the effect of the force readings with the world frame vertical motion 

      # Combine the motions
      # T_move = T_horiz_local @ T_combined_vertical
      T_move = T_horiz_local
      print("T_move: \n", T_move)
      print("T_horiz_local: \n", T_horiz_local)
      # print("T_combined_vertical: \n", T_combined_vertical)
      # Get the target pose
      currentPose = rtde_help.getCurrentPose()
      targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currentPose)
      # Check if the motion is significant to avoid unnecessary motion
      if significant_motion_check(currentPose, targetPose):
        rtde_help.goToPoseAdaptive(targetPose, time=0.05)
    ##################################################
    #                  ROTATION 2                    #
    ##################################################

    ##################################################
    print("time elapsed: ", time.time() - startTime)
    # currentPose = rtde_help.getCurrentPose()
    # print("End pose: ", currentPose)
    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--timeLimit', type=float, help='time limit for the adaptive motion', default= 5)
  parser.add_argument('--pathlLimit', type=float, help='path-length limit for the adaptive motion (m)', default= 0.03)
  parser.add_argument('--normalForce', type=float, help='normal force threshold', default= 1)  
  args = parser.parse_args()    

  main(args)








#  T_rot_step = adpt_help.get_Tmat_RotateInY(direction=1)  # Positive Y-direction
#         targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
#         rtde_help.goToPoseAdaptive(targetPose, time=0.01)

#         currentPose = rtde_help.getCurrentPose()
#         T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
        
#         T_relative = np.linalg.inv(T_start) @ T_curr
#         relative_angle = np.arccos(T_relative[2, 2]) * 180 / np.pi  # Rotation relative to start
        
#         cumulative_rot_angle = relative_angle  # Track cumulative rotation
#         print("Cumulative Rotation angle: ", cumulative_rot_angle)
#         print("Relative angle: ", relative_angle)

#         if cumulative_rot_angle >= 10:
#             rtde_help.stopAtCurrPoseAdaptive()
#             break

## OLD CODE 
# cumulative_rot_angle = 0 # going to accumulate the snout orientation from here
    # Below is storing the end effector pose at the beginning of the adaptive motion, 
    # # motion and force sensing should be relative to this
    # T_lat_originalX = adpt_help.get_Tmat_TranlateInX(direction = -1) 
    # # T_lat_originalY = adpt_help.get_Tmat_TranlateInY(direction = -1) 
    # # T_lat_originalZ = adpt_help.get_Tmat_TranlateInZ(direction = -1)

    # FT_help.setNowAsBias()
    # currentPose = rtde_help.getCurrentPose()
    # T_start = adpt_help.get_Tmat_from_Pose(currentPose)
    # R_start = T_start[:3, :3]  # Rotation matrix of the original frame

    # ########################################
    # while (time.time() - startTime) < args.timeLimit: # trials for 10 seconds
    #   currentPose = rtde_help.getCurrentPose()
    #   T_start = adpt_help.get_Tmat_from_Pose(currentPose)
    #   print("Current.z: ", currentPose.pose.position.z)
    #   startTCPpose = rtde_help.getTCPPose(currentPose)
    #   print("Start TCP pose: ", startTCPpose)

    #   ######################
    #   # ADAPTIVE MOTION WHILE LATERAL MOVEMENT HAPPENS ######### WORKS!
    #   ######################
    #   # $ #
    #   while time.time() - startTime < 2: # 2.5 seconds
    #     currentPose = rtde_help.getCurrentPose()
    #     F_normal = FT_help.averageFz_noOffset
    #     T_normal = adpt_help.get_Tmat_axialMove(F_normal, args.normalForce)
    #     T_move = np.eye(4)
    #     T_move[:3, 3] = [-0.01, 0, T_normal[2, 3]]
        

    #     targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currentPose)
    #     if significant_motion_check(currentPose, targetPose):
    #       rtde_help.goToPoseAdaptive(targetPose, time=0.05)
    #   # $ #


    #     # ## print("Currrent.z: ", currentPose.pose.position.z)
    #     # # adpt_help.d_lat = 30e-3 # 10 mm/s
    #     # currentPose = rtde_help.getCurrentPose()

    #     # T_rot = np.eye(4) # no rotation
    #     # F_normal = FT_help.averageFz_noOffset # in non-rotated frame
    #     # T_normal = adpt_help.get_Tmat_axialMove(F_normal, args.normalForce) # get a matrix for moving in the z direction as a function of the measured normal force and the threshold in args
    #     # T_move =  T_rot @ T_normal @ T_lat_originalX  # move in the x direction and then in the z direction, this is a calculation for the next pose
    #     # # print("T_lat_originalX: \n ", T_lat_originalX)
    #     # # print("T_move: \n", T_move)
        
    #     # targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currentPose) # go from current pose to target pose
    #     # rtde_help.goToPoseAdaptive(targetPose, time =1) # 

    #   # ######################
    #   # #     ROTATION 1     # without any extra motions while rotating
    #   # ######################
    #   # Rotate in the positive direction
    #   # For the first rotation, we don't need to track two different angles. 
    #   overall_angle = 0  # Initialize cumulative rotation angle
    #   ## print("I'm here!")
    #   while overall_angle <= 15:
    #     adpt_help.dw = 0.1 # IMPORTANT PARAMETER AS WELL
    #     currentPose = rtde_help.getCurrentPose()
    #     T_rot_step = adpt_help.get_Tmat_RotateInY(direction=1)

    #     targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
    #     rtde_help.goToPoseAdaptive(targetPose, time=0.05)

    #     currentPose = rtde_help.getCurrentPose()
    #     T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
    #     T_overall = np.linalg.inv(T_start) @ T_curr
    #     overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi

    #     if overall_angle >= 15:
    #       rtde_help.stopAtCurrPoseAdaptive()
    #       break
    #     # adpt_help.dw = 0.1
    #     # T_rot_step = adpt_help.get_Tmat_RotateInY(direction=1)  # Positive Y-direction
    #     # targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
    #     # rtde_help.goToPoseAdaptive(targetPose, time = 1)
    #     # currentPose = rtde_help.getCurrentPose()
    #     # T_curr = adpt_help.get_Tmat_from_Pose(currentPose)

    #     # T_overall = np.linalg.inv(T_start) @ T_curr
    #     # overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi  # Rotation relative to start
        
    #     # # Adjust the angle sign for negative rotation
    #     # if T_overall[2, 0] > 0:  # Check direction of rotation based on off-diagonal terms
    #     #     overall_angle  = -overall_angle 

    #     # # cumulative_rot_angle = relative_angle  # Track cumulative rotation
    #     # #print("Overall angle (relative to coord frame at Pose C): ", overall_angle)

    #     # if overall_angle >= 15:
    #     #     rtde_help.stopAtCurrPoseAdaptive()
    #     #     break
      
    #   #############################################
    #   #     HORIZONTAL MOTION AFTER ROTATION 1  
    #   #############################################
    #  ##  print("I'm here!")
    #   FT_help.setNowAsBias()
    #   R_relative = T_overall[:3,:3] # using this to adjust Fz which has now changed due to rotation
      

    #   currentPose = rtde_help.getCurrentPose()
    #   xAfterRotation = currentPose.pose.position.x
    #   while currentPose.pose.position.x <= xAfterRotation + 0.1:
    #     currentPose = rtde_help.getCurrentPose()
    #     T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
    #     F_normal = FT_help.averageFz_noOffset
    #     F_local = np.array([0,0, F_normal]) # in the local frame
    #     F_originalZ = R_relative @ F_local # in the original frame
        
    #     T_normal = adpt_help.get_Tmat_axialMove(F_originalZ[2], args.normalForce)
    #     T_rot = np.eye(4) # no rotation
    #     T_move = T_rot @ T_normal @ T_lat_originalX
    #     print("\n")
    #     print("--------------------")
    #     [print("T_normal: \n", T_normal), print("T_lat_originalX: \n", T_lat_originalX), 
    #      print("T_rot: \n", T_rot), print("T_start: \n", T_start), print("T_curr: \n", T_curr), print("T_move: \n", T_move)]
    #     print("--------------------")
    #     print("\n")
    #     targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currentPose) 

    #     rtde_help.goToPoseAdaptive(targetPose, time = 0.05) 
    #   ######################
    #   #     ROTATION 2    # without any extra motions while rotating
    #   ######################*
    #   #   # Rotate in the negative direction
    #   # print("I'm here!")
    #   # relative_angle = 0  
    #   # FT_help.setNowAsBias()
    #   # # adpt_help.dw = 0.1
    #   # # a second angle is defined here to track changes from the frame that the tool is in 
    #   # # after the first rotation. We want the difference between this relative angle and
    #   # # the overall angle to be the angle desired. 
    #   # #print("Overall angle: ", overall_angle)
    #   # currentPose = rtde_help.getCurrentPose()
    #   # T_frame2 = adpt_help.get_Tmat_from_Pose(currentPose) # the frame we are starting in 

    #   # while (overall_angle - relative_angle) <= 25:  # Negative limit for opposite rotation
        
    #   #   adpt_help.dw = 0.1
    #   #   T_rot_step =  adpt_help.get_Tmat_RotateInY(direction=-1)  # Negative Y-direction
    #   #   targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
    #   #   rtde_help.goToPoseAdaptive(targetPose, time= 0.5)

    #   #   currentPose = rtde_help.getCurrentPose()

    #   #   T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
    #   #   T_overall = np.linalg.inv(T_start) @ T_curr
    #   #   overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi  # Rotation relative to start

    #   #   T_relative = np.linalg.inv(T_frame2) @ T_curr
    #   #   relative_angle = np.arccos(T_relative[2, 2]) * 180 / np.pi  # Rotation relative to current pose
        
    #   #   # Adjust the angle sign for negative rotation
    #   #   # if T_overall[2, 0] > 0:  # Check direction of rotation based on off-diagonal terms
    #   #   #     overall_angle = -overall_angle

    #   #   # Adjust the angle sign for negative rotation
    #   #   if T_relative[2, 0] > 0:  # Check direction of rotation based on off-diagonal terms
    #   #       relative_angle = -relative_angle
        
    #   #   print("Overall angle in rotation 2: ", overall_angle)
    #   #   # print("Relative angle in rotation 2: ", relative_angle)
    #   #   if (overall_angle - relative_angle) >= 25:  # Stop when desired negative rotation is reached
    #   #       rtde_help.stopAtCurrPoseAdaptive()
    #   #       break





    #   # ######################
    #   # #    ACTUAL ROTATION 2    # without any extra motions while rotating
    #   # ######################*
    #   # #   # Rotate in the negative direction
    #   # relative_angle = 0  
    #   # FT_help.setNowAsBias()
    #   # # adpt_help.dw = 0.1
    #   # # a second angle is defined here to track changes from the frame that the tool is in 
    #   # # after the first rotation. We want the difference between this relative angle and
    #   # # the overall angle to be the angle desired. 
    #   # #print("Overall angle: ", overall_angle)
    #   # currentPose = rtde_help.getCurrentPose()
    #   # T_frame2 = adpt_help.get_Tmat_from_Pose(currentPose) # the frame we are starting in 

    #   # while (np.abs(overall_angle - relative_angle)) >= 5:  # Negative limit for opposite rotation
        
    #   #   adpt_help.dw = 0.1
    #   #   T_rot_step =  adpt_help.get_Tmat_RotateInY(direction=-1)  # Negative Y-direction
    #   #   targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
    #   #   rtde_help.goToPoseAdaptive(targetPose, time= 0.02)

    #   #   currentPose = rtde_help.getCurrentPose()

    #   #   T_curr = adpt_he6lp.get_Tmat_from_Pose(currentPose)
    #   #   T_overall = np.linalg.inv(T_start) @ T_curr
    #   #   overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi  # Rotation relative to start

    #   #   T_relative = np.linalg.inv(T_frame2) @ T_curr
    #   #   relative_angle = np.arccos(T_relative[2, 2]) * 180 / np.pi  # Rotation relative to current pose
        
    #   #   # Adjust the angle sign for negative rotation
    #   #   if T_overall[2, 0] > 0:  # Check direction of rotation based on off-diagonal terms
    #   #       overall_angle = -overall_angle

    #   #   # Adjust the angle sign for negative rotation
    #   #   if T_relative[2, 0] > 0:  # Check direction of rotation based on off-diagonal terms
    #   #       relative_angle = -relative_angle
        
    #   #  # print("Overall angle in rotation 2: ", overall_angle)
    #   #   # print("Relative angle in rotation 2: ", relative_angle)
    #   #   if overall_angle <= 5:  # Stop when desired negative rotation is reached
    #   #       rtde_help.stopAtCurrPoseAdaptive()
    #   #       break

    #   # ############################################
    #   # ##   HORIZONTAL MOTION AFTER ROTATION 2 
    #   # ############################################
    #   # #print("I'm here!")
    #   # FT_help.setNowAsBias()
    #   # R_relative = T_overall[:3,:3] # using this to adjust Fz which has now changed due to rotation

    #   # currentPose = rtde_help.getCurrentPose()
    #   # xAfterRotation = currentPose.pose.position.x
    #   # while currentPose.pose.position.x <= xAfterRotation + 0.05:
    #   #   F_normal = FT_help.averageFz_noOffset
    #   #   F_local = np.array([0,0, F_normal]) # in the local frame
    #   #   F_originalZ = R_relative @ F_local # in the original frame

    #   #   #print("F_normal in local frame: ", F_local[2])
    #   #   #print("F_normal in original frame: ", F_originalZ[2])
    #   #   ## print("Current.z: ", currentPose.pose.position.z)
    #   #   T_normal = adpt_help.get_Tmat_axialMove(F_originalZ[2], args.normalForce)
    #   #   T_rot = np.eye(4) # no rotation
    #   #   T_move =  T_rot @ T_normalcurrentPose = rtde_help.getCurrentPose()
    # # print("End pose: ", currentPose) @ T_lat_originalX 

    #   #   currentPose = rtde_help.getCurrentPose()
    #   #   ####
    #   #   z_position = np.array([0,0, currentPose.pose.position.z]) # in the local frame
    #   #   z_position = R_relative @ z_position # in the original frame
    #   #   print("Z in local frame: ", z_position[2])
    #   #   print("Z in original frame: ", currentPose.pose.position.z)
    #   #   ####
    #   #   targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currentPose) 

    #   #   rtde_help.goToPoseAdaptive(targetPose, time = 0.02)

    # print("Overall angle in rotation 2: ", overall_angle)

    # currentPose = rtde_help.getCurrentPose()
    # z_local = np.array([0,0, currentPose.pose.position.z]) # in the local frame
    # z_global = R_relative @ z_local # in the original frame
    # print("End z: ", z_global[2])

    # startTCPpose = rtde_help.getTCPPose(currentPose)
    # print("Start TCP pose: ", startTCPpose)