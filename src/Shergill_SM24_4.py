#!/usr/bin/env python
# Created by Nimran Shergill (April 21 2025). 
# This is a file for combining planar motion with rotational motion.
# This file is part of the EDG UR10 project.

# Created by Nimran Shergill (September 09 2025)
# This is a file for closed-loop trajectory control for the robot arm in granular media.
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

  F_normalThres = args.normalForce
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

  def RotationBaseCode(T_start, overall_angle, sign):
      adpt_help.dw = 0.01
      #Define and execute motion
      T_rot_step = adpt_help.get_Tmat_RotateInY(direction = sign) 
      currentPose = rtde_help.getCurrentPose()
      targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
      rtde_help.goToPoseAdaptive(targetPose, time=0.05)
      # Update angle
      currentPose = rtde_help.getCurrentPose()
      T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
      T_overall = np.linalg.inv(T_start) @ T_curr 
      new_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi 
      if T_overall[2, 0] > 0:  
        new_angle = -new_angle # making sure it updates, will be "overall_angle" at end

      R_relative = T_overall[:3,:3] 
      return R_relative, new_angle
     

  def Rotate(T_start, overall_angle, sign, delta_rotAngle = 2):
    starting_angle = overall_angle
    
    if sign == -1: 
      while overall_angle > starting_angle - delta_rotAngle: # negative rotation
        R_relative, overall_angle = RotationBaseCode(T_start, overall_angle, sign)

    elif sign == 1:
       while overall_angle < starting_angle + delta_rotAngle:
        R_relative, overall_angle = RotationBaseCode(T_start, overall_angle, sign)
  
    t_horiz_local = R_relative.T @ np.array([-0.01, 0, 0]) 
    Vertical_Axis_Local = R_relative.T @ np.array([0,0,1]) 
    return R_relative, t_horiz_local, Vertical_Axis_Local, overall_angle
    
  def RotationCheck(currentPose, waypoint, desired_vel, overall_angle, beta, T_start, 
                    R_relative, t_horiz_local, Vertical_Axis_Local, delta_rotAngle = 2):
    # This function checks the slope after 2 cm and evaluates whether to rotate
    # then the rotation is performed
    # rotation matrix is defined
    # Checking slope
    # Need condition for over rotation 
    dx = waypoint[0] - currentPose.pose.position.x
    dz = waypoint[2] - currentPose.pose.position.z
    if abs(dx) < 1e-6: # might happen at the end
      print("dx approx. 0, vertical slope case, avoiding rotation")
      return R_relative, t_horiz_local, Vertical_Axis_Local, overall_angle
    
    vel = dz/dx # Calculating slope 
    # Compare with desired_vel and choose rotation direction
    difference = np.abs(desired_vel) - np.abs(vel)
    tolerance = 0.0001 # EDIT ME, original value = 0.0001
    if desired_vel < 0: # waypoint is below x=0
        if difference < tolerance: # less negative number than desired slope
            sign = -1
            if overall_angle - delta_rotAngle < -30: # might add a .5 to allow for wiggle room
               print("Skipping rotaion: would exceed -30 degrees")
               return R_relative, t_horiz_local, Vertical_Axis_Local, overall_angle
            Rotate(T_start, overall_angle, sign)
        elif difference > tolerance: # more negative number than desired slope
            sign = 1
            if overall_angle + delta_rotAngle > 30:
               print("Skipping rotation: would exceed +30 degrees")
               return R_relative, t_horiz_local, Vertical_Axis_Local, overall_angle
            Rotate(T_start, overall_angle, sign)
    return R_relative, t_horiz_local, Vertical_Axis_Local, overall_angle


    # elif desired_vel > 0: # Waypoint is above x = 0 
    #     if difference > tolerance:
    #         # TODO
    #     elif difference < tolerance:
    #         # TODO
    #     else difference = tolerance:
    #         # TODO 
        
    # else: desired_vel = 0:
    #       if difference > tolerance:
    #         # TODO
    #     elif difference < tolerance:
    #         # TODO
    #     else difference = tolerance:
    #         # TODO 
        
        # no rotation

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

    ################################ INITIATE MOTION SEQUENCE ################################
    # Initializing parameters
    currentPose = rtde_help.getCurrentPose()
    T_start = adpt_help.get_Tmat_from_Pose(currentPose) # get the transformation matrix from the current pose
    R_start = T_start[:3,:3] # get the rotation matrix from the transformation matrix
    T_horiz_world = adpt_help.get_Tmat_TranlateInX(direction = -1) # move in the negative x direction
    T_vertical_world = adpt_help.get_Tmat_TranlateInZ(direction = 1) # move in the positive z direction
    T_cumulative = np.eye(4) # cumulative transformation matrix
    T_move = np.eye(4) 
    overall_angle = 0 
    beta = 25; # EDIT ME ###################

    # Define waypoint
    waypoint = [x,y,z]; # EDIT THIS LINE
    args.waypoint = waypoint
    args.startPose = currentPose

    # Calculate thetadot (velocity, slope) from current position to waypoint
    desired_vel = (waypoint[2] - currentPose.pose.position.z)/(waypoint[0] - currentPose.pose.position.x)
    
    #MOTION SEQUENCE BEGINS
    # ZERO GRAVITY AND OTHER FORCES
    FT_help.setNowAsBias() # offset the force sensor, zeros gravity and other forces
    args.ForceOffset1 = [FT_help.offSetFx, FT_help.offSetFy, FT_help.offSetFz, FT_help.offSetTx, FT_help.offSetTy, FT_help.offSetTz]

    input("Press <Enter> to snout motion sequence with horizontal motion + rotations")
    dataLoggerEnable(True)
    rospy.sleep(0.5) # default is 0.5

    # FIRST TWO CM - NO ROTATION YET ########################################
    syncPub.publish(1)
    x_start = currentPose.pose.position.x
    while currentPose.pose.position.x < x_start + 0.02:
      adpt_help.dw = 0.01
      # Vertical adaptive motion 
      Fz = FT_help.averageFz_noOffset
      print("Fz: ", Fz)
      T_normal = adpt_help.get_Tmat_axialMove(Fz, F_normalThres)
      # Combine the motion
      T_move = T_horiz_world @ T_normal

      # Get the target pose 
      targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currentPose)
      rtde_help.goToPoseAdaptive(targetPose, time = 0.5)
      currentPose = rtde_help.getCurrentPose()

      # Fulfill the vertical motion
      tolerance = 0.0010 # I'll adjust this if I start moving into cm territory
      while abs(currentPose.pose.position.z - targetPose.pose.position.z) > tolerance:
        rtde_help.goToPoseAdaptive(targetPose, time = 0.5)
        currentPose = rtde_help.getCurrentPose()
    syncPub.publish(2) # end of the first motion segment

    # First rotation
    R_relative = np.eye(3);
    t_horiz_local = [-0.01, 0, 0]
    Vertical_Axis_Local = [0, 0, 0.01]
    R_relative, t_horiz_local, Vertical_Axis_Local, overall_angle = RotationCheck(currentPose, waypoint, desired_vel, overall_angle, beta,
                                                                                   T_start, R_relative, t_horiz_local, Vertical_Axis_Local)
    
    currentPose = rtde_help.getCurrentPose()
    counter = 3
    while currentPose.pose.position.x < waypoint[0]:
        x_start = currentPose.pose.position.x
        x_end = min(x_start + 0.02, waypoint[0]) # x_start (wherever we are) + 2 cm
        syncPub.publish(counter) #################################
        FT_help.setNowAsBias()
        T_move = np.eye(4) 
        while currentPose.pose.position.x < x_end: # MOTION FOR TWO CM
          F_world = R_relative @ np.array([FT_help.averageFx_noOffset, FT_help.averageFy_noOffset, FT_help.averageFz_noOffset])
          F_vertical_world = np.array([0,0, F_world[2]]) 
          F_vertical_local = R_relative.T @ F_vertical_world
          print("F_vertical_local: ", F_vertical_local[2]) 
          
          T_normal = adpt_help.get_Tmat_axialMove(F_vertical_local[2], F_normalThres)
          t_vertical_local = T_normal[:3, 3]
          magnitude = np.dot(t_vertical_local, Vertical_Axis_Local)
          t_vertical_local = magnitude*Vertical_Axis_Local

          t_move = t_vertical_local + t_horiz_local
          T_move[:3,3] = t_move
          
          # Get the target pose 
          targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currentPose)
          rtde_help.goToPoseAdaptive(targetPose, time = 0.5) # EDIT TIME LINE
          currentPose = rtde_help.getCurrentPose()

          tolerance = 0.001 
          while abs(currentPose.pose.position.z - targetPose.pose.position.z) >= tolerance:
            rtde_help.goToPoseAdaptive(targetPose, time = 0.5) # EDIT TIME LINE
            currentPose = rtde_help.getCurrentPose()     

          syncPub.publish(counter+1) ###############################
          counter += 1
          # Rotate the object before
          R_relative, t_horiz_local, Vertical_Axis_Local, overall_angle = RotationCheck(currentPose, waypoint, desired_vel, overall_angle, beta, 
                                                                                        T_start, R_relative, t_horiz_local, Vertical_Axis_Local) 
          currentPose = rtde_help.getCurrentPose()
   #######################################################################
    dataLoggerEnable(False) 
    rospy.sleep(0.2)

   # Checking if waypoint was reached
    currentPose = rtde_help.getCurrentPose()
    if waypoint[2] == currentPose.pose.position.z:
      print("Waypoint reached!")
    else:
      print("Waypoint not reached!")
      print("Waypoint: ", waypoint)
      print("currentPose: ", currentPose)
      
    args.endPose = currentPose
    # save data and clear the temporary folder
    file_help.saveDataParams(args, appendTxt='beta_'+str(args.beta)+'_DEMO_trial_'+str(args.trialNum)+'_Shergill')
    file_help.clearTmpFolder()
  except rospy.ROSInterruptException:
      return
  except KeyboardInterrupt:
      return  
        # NEED TO ENABLE DATALOGGER

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

# Here is code for adaptive motion if its just straight-line trajectory with rotation happening along the way.
#   # Initializing parameters
#     T_cumulative = np.eye(4) # cumulative transformation matrix
#     T_move = np.eye(4) 
#     overall_angle = 0 
#     T_start = adpt_help.get_Tmat_from_Pose(rtde_help.getCurrentPose()) # get the transformation matrix from the current pose
#     R_start = T_start[:3,:3] # get the rotation matrix from the transformation matrix
#     FT_help.setNowAsBias() # not needed yet, but will be used later

#     currentPose = rtde_help.getCurrentPose() # get the current pose after the motion

#     while overall_angle < 7: 
#       # Task: Move forward and rotate to 5 degrees along the way
#       # Define some conditional to stop doing the joint motion
#       # To create motion:
#       # Define motion matrix
#       currentPose = rtde_help.getCurrentPose() # get the current pose after the motion
#       T_curr = adpt_help.get_Tmat_from_Pose(currentPose) # get the transformation matrix from the current pose
#       R_local = T_curr[:3,:3] # get the rotation matrix from the transformation matrix
#       print("R_local: ", R_local)

#       F_world = R_local @ np.array([FT_help.averageFx_noOffset, FT_help.averageFy_noOffset, FT_help.averageFz_noOffset]) # force in the world frame
#       T_vertical = adpt_help.get_Tmat_axialMovement(F_world[2], F_normalThres) # get the transformation matrix for the vertical movement
#       print("T_vertical: ", T_vertical)

#       adpt_help.dw = 0.05 # slowing down the rotation speed
#       T_rot_step = adpt_help.get_Tmat_RotateInY(direction=1) # rotate in y direction, about the 
#       T_horizontal_step = adpt_help.get_Tmat_TranlateInX(direction=-1) # move in x direction 
#       T_vertical_step = adpt_help.get_Tmat_TranlateInZ(direction=1) # move in z direction, not used here but can be used later
#       # print("T_rot_step: ", T_rot_step)
#       # print("T_horizontal_step: ", T_horizontal_step)
#       # print("T_vertical_step: ", T_vertical_step)

#       T_step = T_horizontal_step @ T_vertical @ T_rot_step
#       T_cumulative = T_cumulative @ T_step
#       T_target = T_start @ T_cumulative # get the target pose from the transformation matrix and the starting pose
#       targetPose_New = adpt_help.get_PoseStamped_from_T_initPose(T_target, currentPose) # get the target pose from the transformation matrix and the current pose
#       print("T_target: ", T_target)
#       print("Target pose: ", targetPose_New)


#       # Does not consider motion strictly in the global frame, but rather in the local frame of the robot
#       # both T_move and T_move2 work
#       T_move = T_horizontal_step @ T_rot_step # T_move = Translation * Rotation * Scaling
#       T_trans = T_horizontal_step @ T_vertical_step
#       print("T_trans: ", T_trans)
#       print("T_rot_step: ", T_rot_step)
#       T_move2 = T_trans @ T_rot_step # T_move = Translation * Rotation * Scaling
#       print("T_move2: ", T_move2)

#       # TODO: projecting back to the global frame

#       targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move2, currentPose) # get the target pose from the transformation matrix and the current pose
#       rtde_help.goToPoseAdaptive(targetPose, time=2) # move to the target pose
      
#       currentPose = rtde_help.getCurrentPose() # get the current pose after the motion
#       T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
#       T_overall = np.linalg.inv(T_start) @ T_curr 
#       overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi 
#       if T_overall[2, 0] > 0:  
#           overall_angle = -overall_angle
#       print("Current angle: ", overall_angle)
#       print('Current x position: ', currentPose.pose.position.x)
#     #rtde_help.stopAtCurrPoseAdaptive() # stop at the current pose

#while overall_angle < 7: 
    #   adpt_help.dw = 0.01 # slowing down the rotation speed
    #   # Task: Move forward and rotate to 5 degrees along the way
    #   # Define some conditional to stop doing the joint motion
    #   # To create motion:
    #   # Define motion matrix
    #   currentPose = rtde_help.getCurrentPose() # get the current pose after the motion
    #   T_curr = adpt_help.get_Tmat_from_Pose(currentPose) # get the transformation matrix from the current pose
    #   R_local = T_curr[:3,:3] # get the rotation matrix from the transformation matrix
    #   # print("R_local: ", R_local)

    #   F_world = R_local @ np.array([FT_help.averageFx_noOffset, FT_help.averageFy_noOffset, FT_help.averageFz_noOffset]) # force in the world frame
    #   T_vertical = adpt_help.get_Tmat_axialMove(F_world[2], F_normalThres) # get the transformation matrix for the vertical movement
    #   print("T_vertical: ", T_vertical)

    #   T_rot_step = adpt_help.get_Tmat_RotateInY(direction=1) # rotate in y direction, about the 
    #   T_horizontal_step = adpt_help.get_Tmat_TranlateInX(direction=-1) # move in x direction 
    #   T_vertical_step = adpt_help.get_Tmat_TranlateInZ(direction=1) # move in z direction, not used here but can be used later

    #   # T_step = T_horizontal_step @ T_vertical @ T_rot_step
    #   # T_cumulative = T_cumulative @ T_step
    #   # T_target = T_start @ T_cumulative # get the target pose from the transformation matrix and the starting pose
    #   # targetPose_New = adpt_help.get_PoseStamped_from_T_initPose(T_target, currentPose) # get the target pose from the transformation matrix and the current pose
    #   # print("T_target: ", T_target)
    #   # print("Target pose: ", targetPose_New)

    #   # both T_move and T_move2 work
    #   #T_move = T_horizontal_step @ T_rot_step # T_move = Translation * Rotation * Scaling
    #   # T_trans = T_horizontal_step @ T_vertical_step
    #   T_trans2 = T_horizontal_step @ T_vertical 
    #   # print("PREVIOUS T_trans: ", T_trans)
    #   # print(" PROPOSED T_trans2: ", T_trans2)
    #   #T_move2 = T_trans @ T_rot_step # T_move = Translation * Rotation * Scaling
    #   T_move3 = T_trans2 @ T_rot_step # T_move = Translation * Rotation * Scaling
    #   print("T_move3: ", T_move3)

    #   # ***********************************************************************************
    #   # ***********************************************************************************
    #   targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move3, currentPose) # get the target pose from the transformation matrix and the current pose
    #   rtde_help.goToPoseAdaptive(targetPose, time=2) # move to the target pose 
    #   # ***********************************************************************************
    #   # ***********************************************************************************

    #   currentPose = rtde_help.getCurrentPose() # get the current pose after the motion
    #   T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
    #   T_overall = np.linalg.inv(T_start) @ T_curr 
    #   overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi 
    #   if T_overall[2, 0] > 0:  
    #       overall_angle = -overall_angle
    #   print("Current angle: ", overall_angle)
    #   print('Current x position: ', currentPose.pose.position.x)
    # #rtde_help.stopAtCurrPoseAdaptive() # stop at the current pose
