#!/usr/bin/env python
## Shergill_Snout_Motion_2024
# Last updated: 10/31/2024
# Adapted from Jungpyo's simple_robot_control.py script
# Eventually to add the data collection part but for now just testing the first few motion steps

###########################################################################



# Authors: Jungpyo Lee
# Create: Oct.08.2024
# Last update: Oct.10.2024
# Description: This script is primarily for basic robot control using UR10e robot. 
# It first moves to initial position (position A), then it move position A (poseA) to B (poseB) and rotate by 45 degrees in y-axis (poseC).

#!/usr/bin/env python

# Authors: Jungpyo Lee
# Create: Nov.04.2024
# Last update: Nov.04.2024
# Description: This script is primarily for adaptive robot control using UR10e robot. 
# It first moves to initial position (position A), then it move position A (poseA) to B (poseB) and preform adaptive robot control.
# It basically moves to the right and move up and down depending on the force feedback. If the force is negative, it moves up, otherwise it moves down.

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
  adpt_help = adaptMotionHelp(d_w = 1,d_lat = 10e-3, d_z= 5e-3)

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


  # Set the start pose
  # Pose in the XY direction
  # Note the new coordinates: x is pointing to us, y is pointing to the left, and z is pointing down is when 
  # we do tf.transformations.quaternion_from_euler(np.pi, 0,-np.pi/2,'sxyz')
  # ORIGINAL COORD SYSTEM: RHR - x axis is pointing right, y axis is point out page, and z axis is pointing up 
  
  # This first chunk of code is getting the current pose of the robot and setting the start position and orientation 
  # We need the current pose to have xy motion without z motion 
  # We also rotate the axes. THIS IS IMPORTANT. The rotation in the z is done as is so the snout is orientated correctly 
  # in tje 'horizontal' direction. 
  # The new axes are x-axis: left, y-axis: out of the page, z-axis: down
  
  currentPose = rtde_help.getCurrentPose()
  startPositionA = [0.320, -0.200, currentPose.pose.position.z] # change the first two parameters to be the "beginning of the tank"
  startOrientationA = tf.transformations.quaternion_from_euler(np.pi, 0,-np.pi,'sxyz') #static (s) rotating (r)
  # Note the new coordinates: x is pointing to us, y is pointing to the left, and z is pointing down.
  startPoseA = rtde_help.getPoseObj(startPositionA, startOrientationA)

# We descend into media. No rotation. 
  startPositionB = [0.320, -0.200, 0.3]
  startOrientationB = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi,'sxyz') # not moving it from the previous transformation
  startPoseB = rtde_help.getPoseObj(startPositionB, startOrientationB) 

  # Creating new objects to use for the rotation
  # d_w is a speed paramter. It is the speed of the rotation in radians per second.
  # d_lat is a speed parameter. It is the speed of the lateral motion in meters per second.
  # d_z is a speed parameter. It is the speed of the vertical motion in meters per second.
  RotationA = adaptMotionHelp()
  RotationA.d_w = 0.05; RotationA.d_lat = 10e-3; RotationA.d_z = 5e-3
  RotationB = adaptMotionHelp() 
  RotationB.d_w = 20; RotationB.d_lat = 10e-3; RotationB.d_z = 5e-3

  # try block so that we can have a keyboard exception
  try:

# Initial position 
    input("Press <Enter> to go to startPoseA")
    rtde_help.goToPose(startPoseA)
    rospy.sleep(1)
    input("Press <Enter> to go to startPoseB")
    rtde_help.goToPose(startPoseB)
    rospy.sleep(1)

    # offset the force sensor - this step is for the force readings to make sense right?
    FT_help.setNowAsBias()


    input("Press <Enter> to adaptive robot control")
    startTime = time.time()
    
    rot_angle = 0
    # loop for adaptive robot control for args.timeLimit seconds
    while (time.time() - startTime) < args.timeLimit:
      # Get the current pose
      currentPose = rtde_help.getCurrentPose()

      # Get the next pose
      # lateral
      T_lat = adpt_help.get_Tmat_TranlateInZ(direction = 1)
      # rotation
      T_start = adpt_help.get_Tmat_from_Pose(currentPose)
      
      while rot_angle <= 25:
        # I may have something flipped in the reasoning for the axes below,
        #  can proceed with work by using -1 direction but should correct
        #  for accurateness and completeness
        T_rot = RotationA.get_Tmat_RotateInY(direction = 1) # CHANGED FROM - TO +
        targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_rot, currentPose)
        rtde_help.goToPoseAdaptive(targetPose, time=2)
        currentPose = rtde_help.getCurrentPose()
        T_curr = adpt_help.get_Tmat_from_Pose(currentPose)
        T_rot = T_start @ T_curr
        rot_angle = np.arccos(T_rot[2,2]) * 180/np.pi
        #print("Rotation angle: ", rot_angle)
        #print("Rotation angle: ", rot_angle<=25)
        if rot_angle >= 25:
          rtde_help.stopAtCurrPoseAdaptive()
          break
      

      # T_rot = np.eye(4) # no rotation
      
      # T_lat = adpt_help.get_Tmat_TranlateInZ(direction = 1)
      # # nor+mal
      # F_normal = FT_help.averageFz_noOffset # if axes change, it's no longer z then?
      # print('F_normal: ', F_normal)
      # T_normal = adpt_help.get_Tmat_axialMove(F_normal, args.normalForce)
      # T_move = T_lat @ T_normal @ T_rot
      # currPose = rtde_help.getCurrentPose()
      # targetPose = adpt_help.get_PoseStamped_from_T_initPose(T_move, currPose)

      # # Move to the next pose -- may tune depending on the motion I'm seeing
      # rtde_help.goToPoseAdaptive(targetPose, time = 0.1)


    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--timeLimit', type=float, help='time limit for the adaptive motion', default= 8)
  parser.add_argument('--pathlLimit', type=float, help='path-length limit for the adaptive motion (m)', default= 0.5)
  parser.add_argument('--normalForce', type=float, help='normal force threshold', default= 0.25)  
  args = parser.parse_args()    

  main(args)



    # def get_Tmat_RotateInY(self, direction = 1):
    #     rot_axis = np.array([0.0, np.sign(direction), 0.0])
    #     omega_hat = hat(rot_axis)
    #     Rw = scipy.linalg.expm(self.dw* omega_hat)
    #     return create_transform_matrix(Rw, [0,0,0]




###############################################################################
