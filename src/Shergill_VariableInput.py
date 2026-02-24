#!/usr/bin/env python
# Created by Nimran Shergill 02 12th 2026. In this code, the end effector has different programmed initial 
# conditions (a certain trajectory) leading to a certain waypoint. From there, force control is enabled to observe
# the extent to which the system is history-dependent and whether depth had played a more significant role in previous
# studies than previously understood. 

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
    
  # Set force threshold
  F_normalThres = args.normalForce
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
  beforePose = currentPose  
  PositionA = [currentPose.pose.position.x, currentPose.pose.position.y, 0.350]
  OrientationA = [currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w]
  PoseA = rtde_help.getPoseObj(PositionA, OrientationA)

  # Pose B has to be defined relative to A so it is defined during the motion sequence

  # We descend into media. No rotation. EDIT Z BELOW TO CHANGE THE INITIAL DEPTH
  PositionC = [0.410, -0.230, 0.28] # approx 8 cm below surface of grains, 0.26?
  OrientationC = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi,'sxyz') # not moving it from the previous transformation
  PoseC = rtde_help.getPoseObj(PositionC, OrientationC) 

################ REACH STARTING POSITION ########################
  try:
    # POSE A
    input("Press <Enter> to go to PoseA")
    rtde_help.goToPose(PoseA) 
    rospy.sleep(1)

    # POSE B
    input("Press <Enter> to go to PoseB")
    currentPose = rtde_help.getCurrentPose()
    PositionB = [0.410, -0.230, currentPose.pose.position.z] # change the first two parameters to be the "beginning of the tank"
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
    # NOTES: To change Pose D, need to change x and z positions, so should decide on some slope 
    print('\n')
    input("Press <Enter> to go to PoseD")
    currentPose = rtde_help.getCurrentPose()
    PositionD = [0.450, currentPose.pose.position.y, 0.28] # approx 8 cm below surface of grains
    OrientationD = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi,'sxyz') # not moving it from the previous transformation 
    PoseD = rtde_help.getPoseObj(PositionD, OrientationD) 
    rtde_help.goToPose(PoseD)
    rospy.sleep(1)

    # ZERO GRAVITY AND OTHER FORCES
    FT_help.setNowAsBias() # offset the force sensor, zeros gravity and other forces
    args.ForceOffset1 = [FT_help.offSetFx, FT_help.offSetFy, FT_help.offSetFz, FT_help.offSetTx, FT_help.offSetTy, FT_help.offSetTz]

#######################################################################################
########################### Starting Force Control
#######################################################################################
    input("Press <Enter> to snout motion sequence with horizontal motion + rotations")
    dataLoggerEnable(True)
    currentPose = rtde_help.getCurrentPose()
    T_start = adpt_help.get_Tmat_from_Pose(currentPose) # world frame
    R_original = T_start[:3, :3]  # Rotation matrix of the world frame
    T_horiz_world = adpt_help.get_Tmat_TranlateInX(direction = -1) # move in the negative x direction
    T_vertical_world = adpt_help.get_Tmat_TranlateInZ(direction = 1) # move in the positive z direction
    T_move = np.eye(4) # initialize the move vector outside loop
    overall_angle = 0 # Initialize cumulative rotation angle 
    args.RotationMatrices = [] # in list format
    print("R_original HORIZ 1: ") 
    formatted_rows = [" , ".join(f"{val:.6f}" for val in row) for row in R_original] # Print in MATLAB-like format 
    formatted_rows = ' ; '.join(formatted_rows)
    print(formatted_rows) 
    args.RotationMatrices.append(formatted_rows)
    ################################################################################
    #################### ROTATION FIRST ###########################################
    ################################################################################
    while overall_angle < 22.5: # EDIT THIS LINE
        adpt_help.dw = 0.01
        T_rot_step = adpt_help.get_Tmat_RotateInY(direction=1) # EDIT LINE: 
        currentPose = rtde_help.getCurrentPose()
        targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot_step, currentPose)
        rtde_help.goToPoseAdaptive(targetPose, time=0.05)
        
        currentPose = rtde_help.getCurrentPose()
        T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
        T_overall = np.linalg.inv(T_start) @ T_curr 
        overall_angle = np.arccos(T_overall[2, 2]) * 180 / np.pi 
        if T_overall[2, 0] > 0:  
            overall_angle = -overall_angle

    R_relative = T_overall[:3,:3] 
    T_move = np.eye(4) 
    t_horiz_local = R_relative.T @ np.array([-0.01, 0, 0]) 
    Vertical_Axis_Local = R_relative.T @ np.array([0,0,1]) 
    formatted_rows = [" , ".join(f"{val:.6f}" for val in row) for row in R_relative] # Print in MATLAB-like format 
    formatted_rows = ' ; '.join(formatted_rows)
    print(formatted_rows) 
    args.RotationMatrices.append(formatted_rows)
    print("overall_angle after rotation 2: ", overall_angle)

    rospy.sleep(2)
    FT_help.setNowAsBias() 
    args.ForceOffset3 = [FT_help.offSetFx, FT_help.offSetFy, FT_help.offSetFz, FT_help.offSetTx, FT_help.offSetTy, FT_help.offSetTz]

    currentPose = rtde_help.getCurrentPose()
    starting_x = currentPose.pose.position.x

    syncPub.publish(1) # 1
    while currentPose.pose.position.x < starting_x + 0.2: # EDIT THIS LINE 
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

    syncPub.publish(2) # end of the first motion segment
      
# # #  ######################################## IF NO BEGINNING ROTATION ########################################
    # starting_x = currentPose.pose.position.x
    # syncPub.publish(1)
    # while currentPose.pose.position.x < starting_x + 0.1:
    #   adpt_help.dw = 0.01
    #   # Vertical adaptive motion 
    #   Fz = FT_help.averageFz_noOffset
    #   print("Fz: ", Fz)
    #   T_normal = adpt_help.get_Tmat_axialMove(Fz, F_normalThres)
    #   # Combine the motion
    #   T_move = T_horiz_world @ T_normal

    #   # Get the target pose 
    #   targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currentPose)
    #     #print(" ###################### targetPose z: ", targetPose.pose.position.z)
    #   rtde_help.goToPoseAdaptive(targetPose, time = 0.5)
    #   currentPose = rtde_help.getCurrentPose()

    #   # Fulfill the vertical motion
    #   tolerance = 0.0010 # I'll adjust this if I start moving into cm territory
    #     #print('difference: ', abs(currentPose.pose.position.z - targetPose.pose.position.z))
    #   while abs(currentPose.pose.position.z - targetPose.pose.position.z) > tolerance:
    #     rtde_help.goToPoseAdaptive(targetPose, time = 0.5)
    #     currentPose = rtde_help.getCurrentPose()
    # syncPub.publish(2) # end of the first motion segment


  #######################################################################
    dataLoggerEnable(False) 
    rospy.sleep(0.2)

    currentPose = rtde_help.getCurrentPose()
    print("End x: ", currentPose.pose.position.x)
    print("Before run x: ", beforePose.pose.position.x)
    print("End z: ", currentPose.pose.position.z)
    print("============ Python UR_Interface demo complete!")

    # save data and clear the temporary folder
    file_help.saveDataParams(args, appendTxt='beta_'+str(args.beta)+'_VariableInput_trial_'+str(args.trialNum)+'_Shergill_Snout_Experiment')
    file_help.clearTmpFolder()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  

if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--normalForce', type=float, help='normal force threshold', default=0.5)
  parser.add_argument('--beta', type=int, help='beta angle of wedge', default= 0) # wrote zero so I realize I didn't change this parameter
  parser.add_argument('--trialNum', type=int, help='Trial number', default= 0)
  args = parser.parse_args()    

  main(args)

