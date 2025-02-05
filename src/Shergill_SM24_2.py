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
# Last update: see Git repo
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
from helperFunction.adaptiveMotion import adaptMotionHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.rtde_helper import rtdeHelp




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

  # class PID:
  #   def __init__(self, setpoint, kp=0.0, ki=0.0, kd=0.0):
  #     self.kp = kp
  #     self.ki = ki
  #     self.kd = kd
  #     self.setpoint = setpoint
  #     self.prev_error = 0
  #     self.integral = 0

  #     self.prev_time = time.time()

  #   def update(self, current_position):

  #     error = self.setpoint - current_position
  #     self.integral += error
  #     derivative = error - self.prev_error

  #     current_time = time.time()
  #     dt = current_time - self.prev_time

  #     self.prev_error = error
  #     self.prev_time = current_time

  #     return self.kp * error + self.ki * self.integral * dt + self.kd * derivative / dt

  def low_pass_filter(new_value, previous_value, alpha=0.1):
    return alpha * new_value + (1 - alpha) * previous_value
    
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

  # Set the synchronization Publisher
  syncPub = rospy.Publisher('sync', Int8, queue_size=1)
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
  beforePose = currentPose  
  PositionA = [currentPose.pose.position.x, currentPose.pose.position.y, 0.4]
  OrientationA = [currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w]
  PoseA = rtde_help.getPoseObj(PositionA, OrientationA)

  # We descend into media. No rotation. 
  PositionC = [0.320, -0.200, 0.12]
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
    args.ForceOffset1 = [FT_help.offSetFx, FT_help.offSetFy, FT_help.offSetFz, FT_help.offSetTx, FT_help.offSetTy, FT_help.offSetTz]

    ########################################
    ########################################
    # ADAPTIVE MOTION
    input("Press <Enter> to snout motion sequence with adaptive control")
    dataLoggerEnable(True)

    startTime = time.time() 
    currentPose = rtde_help.getCurrentPose()
    # PRINT X VALUE HERE ############
    T_start = adpt_help.get_Tmat_from_Pose(currentPose) # world frame
    R_start = T_start[:3, :3]  # Rotation matrix of the world frame
    T_horiz_world = adpt_help.get_Tmat_TranlateInX(direction = -1) # move in the negative x direction
    T_vertical_world = adpt_help.get_Tmat_TranlateInZ(direction = 1) # move in the positive z direction

    overall_angle = 0 # Initialize cumulative rotation angle 
    ########################################
    ##################################################
    # ADAPTIVE MOTION WHILE LATERAL MOVEMENT HAPPENS #
    ##################################################
   
    currentPose = rtde_help.getCurrentPose()
    current_x = currentPose.pose.position.x
    # print("current-x before lateral motion: ", currentPose.pose.position.x)
    syncPub.publish(1)
    while currentPose.pose.position.x < current_x + 0.02:
      if currentPose.pose.position.x >= current_x + 0.02:
        rtde_help.stopAtCurrPoseAdaptive()
        break
      # Vertical adaptive motion 
      Fz = FT_help.averageFz_noOffset
      #  print("Fz: ", Fz)
      T_normal = adpt_help.get_Tmat_axialMove(Fz, F_normalThres)
      # Combine the motion
      T_move = T_horiz_world @ T_normal

      targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currentPose)
      print('target-z: ', targetPose.pose.position.z)
      rtde_help.goToPoseAdaptive(targetPose, time = 0.5)
      currentPose = rtde_help.getCurrentPose()
      # while currentPose.pose.position.z != targetPose.pose.position.z:
      #   rtde_help.goToPoseAdaptive(targetPose, time = 0.5)
      #   currentPose = rtde_help.getCurrentPose()
      #   print("In the loop-z:", currentPose.pose.position.z)
      print("current-z:", currentPose.pose.position.z) 
    

    syncPub.publish(2)
    #####################################
      # Check if the motion is significant to avoid unnecessary motion
      # if significant_motion_check(currentPose, targetPose):
      #   rtde_help.goToPoseAdaptive(targetPose, time=0.05)

      # pid = PID(targetPose.pose.position.z, kd = 0.0005)
      # while True:
      #   currentPose = rtde_help.getCurrentPose()
      #   control_output = pid.update(currentPose.pose.position.z)
      #   targetPose.pose.position.z += control_output
      #   print("Control output: ", control_output)
      #   rtde_help.goToPoseAdaptive(targetPose, time=0.05)
      #  # time.sleep(0.1)

      # print("current z after lateral motion: ", currentPose.pose.position.z)

  ##################################################
  #                  ROTATION 1                    #
  ##################################################
    currentPose = rtde_help.getCurrentPose()
    print("current-x after lateral motion: ", currentPose.pose.position.x)
    print("Rotation 1") 
    adpt_help.dw = 0.01
    while overall_angle <= 12.5: # positive rotation
      if overall_angle >= 12.5:
        rtde_help.stopAtCurrPoseAdaptive()
        break
      T_rot_step = adpt_help.get_Tmat_RotateInY(direction=1) # Positive Y-direction 
      targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
      rtde_help.goToPoseAdaptive(targetPose, time=0.05)
      
      # Updating the overall angle
      currentPose = rtde_help.getCurrentPose()
      T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
      T_overall = np.linalg.inv(T_start) @ T_curr # local frame to the world frame
      # Angle relative to the start
      overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi # may want to use a different angle
      print("Overall angle: ", overall_angle)
      currentPose = rtde_help.getCurrentPose()


  #     #############################################
  #     # Not needed here since we know the direction of rotation and want a final positive angle, will be useful in the future
  #     # if T_overall[2, 0] > 0:  # Check direction of rotation based on off-diagonal terms
  #     #   overall_angle = -overall_angle

  #     # # Condition to break the loop
  #     # if overall_angle >= 12.5:
  #     #   rtde_help.stopAtCurrPoseAdaptive()
  #     #   break
  # #  #######################################################################################
  # #  #  ADAPTIVE HORIZONTAL MOTION AFTER ROTATION 1  --> WORLD FRAME MOTION IN LOCAL FRAME #
  # #  # #######################################################################################

    print("Adaptive motion after rotation 1")
    # PRINT OVERALL ANGLE
    R_relative = T_overall[:3,:3] 
    # tvec_horiz_world = np.array([-0.01, 0, 0]) # move in the negative x direction in the world frame
    t_horiz_local = R_relative.T @ np.array([-0.01, 0, 0]) # translation VECTOR for horizontal motion
    T_move = np.eye(4) # initialize the move vector outside loop
    Vertical_Axis_Local = R_relative.T @ np.array([0,0,1]) # world vertical axis is given by this, useful for projection in loop

    rospy.sleep(1.5)
    FT_help.setNowAsBias() # zero gravity and other forces
    args.ForceOffset2 = [FT_help.offSetFx, FT_help.offSetFy, FT_help.offSetFz, FT_help.offSetTx, FT_help.offSetTy, FT_help.offSetTz]

   # Preparing for the next horizontal segment
    currentPose = rtde_help.getCurrentPose()
    current_x = currentPose.pose.position.x
    syncPub.publish(3)
    while currentPose.pose.position.x <= current_x + 0.02:
      # Horizontal motion defined in a TRANSLATION VECTOR previously outside of this loop
      # Dealing with the vertical motion in the local frame after rotation

      # ISOLATING THE Z-COMPONENT OF THE FORCE VECTOR IN THE WORLD FRAME
      F_world = R_relative @ np.array([FT_help.averageFx_noOffset, FT_help.averageFy_noOffset, FT_help.averageFz_noOffset])
      F_vertical_world = np.array([0,0, F_world[2]]) # z-component has now been isolated
      # Transform from world back to local frame
      F_vertical_local = R_relative.T @ F_vertical_world
      
      # OBTAIN ADAPTIVE MOTION IN WORLD FRAME TRANSFORMED FOR MOTION IN LOCAL FRAME
      # Define transformation vector 
      T_normal = adpt_help.get_Tmat_axialMove(F_vertical_local[2], F_normalThres)
      t_vertical_local = T_normal[:3, 3]
      # Align Motion With Local Vertical Axis Thereby "Correcting" It
      magnitude = np.dot(t_vertical_local, Vertical_Axis_Local) # projection of the motion vector onto the vertical axis (defined outside while loop)
      t_vertical_local = magnitude*Vertical_Axis_Local

      # Combine the motions
      t_move = t_vertical_local + t_horiz_local
      T_move[:3,3] = t_move
    
      # Get the target pose
      targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currentPose)
      # Check if the motion is significant to avoid unnecessary motion
      # if significant_motion_check(currentPose, targetPose):
      rtde_help.goToPoseAdaptive(targetPose, time=0.5)

      currentPose = rtde_help.getCurrentPose()
              
  #   # ##################################################
  #   # #                  ROTATION 2                    #
  #   # ##################################################
  #   syncPub.publish(4) 
  #   currentPose = rtde_help.getCurrentPose()   
  #   print("current z after rotation 1: ", currentPose.pose.position.z)      
    # rospy.sleep(1.5)  
    # while overall_angle >= -12.5: # negative rotation
    #   adpt_help.dw = 0.01
    #   T_rot_step = adpt_help.get_Tmat_RotateInY(direction=-1) # Positive Y-direction  
    #   currentPose = rtde_help.getCurrentPose()
    #   targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
    #   rtde_help.goToPoseAdaptive(targetPose, time=0.05)
      
    #   # Updating the overall angle
    #   currentPose = rtde_help.getCurrentPose()
    #   T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
    #   T_overall = np.linalg.inv(T_start) @ T_curr # local frame to the world frame
    #   # Angle relative to the start
    #   overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi # may want to use a different angle
    
    #   # Not needed here since we know the direction of rotation and want a final positive angle, will be useful in the future
    #   if T_overall[2, 0] > 0:  # Check direction of rotation based on off-diagonal terms
    #     overall_angle = -overall_angle

    #   # print("Overall angle: ", overall_angle)
    #   # Condition to break the loop
    #   if overall_angle <= -12.5:
    #     rtde_help.stopAtCurrPoseAdaptive()
    #     break
    # ##################################################
    # #   ADAPTIVE HORIZONTAL MOTION AFTER ROTATION 2  #
    # ##################################################

    # rospy.sleep(2) 

    # FT_help.setNowAsBias() # zero gravity and other forces
    # R_relative = T_overall[:3,:3] 

    # # Dealing with the horizontal motion in the local frame after rotation
    # tvec_horiz_world = np.array([-0.01, 0, 0]) # move in the negative x direction in the world frame
    # t_horiz_local = R_relative.T @ tvec_horiz_world # translation VECTOR for horizontal motion

    # # Preparing for the next horizontal segment
    # currentPose = rtde_help.getCurrentPose()
    # current_x = currentPose.pose.position.x
    # print("current z after rotation 1: ", currentPose.pose.position.z)

    # # print("currentPose start of rotation: =============== \n", currentPose)
    # while currentPose.pose.position.x <= current_x + 0.06:
    #   # Horizontal motion defined in a TRANSLATION VECTOR previously outside of this loop
    #   # Dealing with the vertical motion in the local frame after rotation
    #   # VERTICAL MOTION FROM ALL COMPONENTS
    #   # Get the force vector
    #   Fx = FT_help.averageFx_noOffset
    #   Fy = FT_help.averageFy_noOffset
    #   Fz = FT_help.averageFz_noOffset
    #   # Define a force vector
    #   F_local = np.array([Fx, Fy, Fz])
    #   # Transform to world frame
    #   F_world = R_relative @ F_local
    #   F_vertical_world = np.array([0,0, F_world[2]]) # z-component has now been isolated
    #   # Transform from world back to local frame
    #   F_vertical_local = R_relative.T @ F_vertical_world
    #   # print("F_local: ", F_local)
    #   print("========================================")
    #   print("F_vertical_local: ", F_vertical_local[2])

    #   # Define transformation vector 
    #   T_normal = adpt_help.get_Tmat_axialMove(F_vertical_local[2], F_normalThres)
    #   t_vertical_local = T_normal[:3, 3]
    #   # Transform to World Coordinates
    #   t_vertical_world = R_relative @ t_vertical_local
    #   # Project Motion Back to Local Vertical Axis Thereby "Correcting" It
    #   Vertical_Axis_Local = R_relative.T @ np.array([0,0,1]) # world vertical axis is given by this 
    #   magnitude = np.dot(t_vertical_local, Vertical_Axis_Local)
    #   t_vertical_local = magnitude*Vertical_Axis_Local

    #   # Combine the motions
    #   t_move = t_vertical_local + t_horiz_local
    #   T_move = np.eye(4)
    #   T_move[:3,3] = t_move
    #   # T_move = T_horiz_local
    #   # print("T_move: \n", T_move)
        
    #   # Get the target pose
    #   currentPose = rtde_help.getCurrentPose()
    #   targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currentPose)
    #   # Check if the motion is significant to avoid unnecessary motion
    #   if significant_motion_check(currentPose, targetPose):
    #     rtde_help.goToPoseAdaptive(targetPose, time=0.05)

    #   currentPose = rtde_help.getCurrentPose()

    dataLoggerEnable(False)
    rospy.sleep(0.2)
    ##################################################

  #   currentPose = rtde_help.getCurrentPose()
  #  # print("End pose: =============== \n", currentPose)
  #   print("overall_angle: ", overall_angle)
  #   print("ending z (after rotation 2): ", currentPose.pose.position.z)
  #   current_x = currentPose.pose.position.x

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

  #   # Convert to degrees
  #   roll_deg = np.degrees(roll)
  #   pitch_deg = np.degrees(pitch)
  #   yaw_deg = np.degrees(yaw)

  #   print(f"Roll (x-axis): {roll_deg:.2f} degrees")
  #   print(f"Pitch (y-axis): {pitch_deg:.2f} degrees")
  #   print(f"Yaw (z-axis): {yaw_deg:.2f} degrees")
  #   print("time elapsed: ", time.time() - startTime)
    # currentPose = rtde_help.getCurrentPose()
    # print("End pose: ", currentPose)
    currentPose = rtde_help.getCurrentPose()
    print("End x: ", currentPose.pose.position.x)
    print("Before run x: ", beforePose.pose.position.x)
    print("============ Python UR_Interface demo complete!")

    # save data and clear the temporary folder
    file_help.saveDataParams(args, appendTxt='Shergill_Snout_Experiment'+'beta_'+str(args.beta)+'_rot1_'+str(args.rotationAngle1) 
                              + '_rot2_'+str(args.rotationAngle2) + '_trial_'+str(args.trialNum))
    file_help.clearTmpFolder()

    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--timeLimit', type=float, help='time limit for the adaptive motion', default= 5)
  parser.add_argument('--pathlLimit', type=float, help='path-length limit for the adaptive motion (m)', default= 0.01)
  parser.add_argument('--normalForce', type=float, help='normal force threshold', default=0.5)
  parser.add_argument('--beta', type=float, help='beta angle of wedge', default= 0) # wrote zero so I realize I didn't change this parameter
  parser.add_argument('--rotationAngle1', type=float, help='Rotation angle 1', default= 12.5)
  parser.add_argument('--rotationAngle2', type=float, help='Rotation angle 2', default= -12.5)
  parser.add_argument('--trialNum', type=int, help='Trial number', default= 1)
  args = parser.parse_args()    

  main(args)

