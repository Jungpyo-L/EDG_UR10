#!/usr/bin/env python
try:
  import rospy
  import tf
  ros_enabled = True
except:
  print('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
  ros_enabled = False

import numpy as np
from geometry_msgs.msg import PoseStamped
from .utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix, normalize, hat

def get_ObjectPoseStamped_from_T(self,T):   #transformation
        thisPose = PoseStamped()
        thisPose.header.frame_id = "base_link"
        R = T[0:3,0:3]
        quat = quaternion_from_matrix(R)   #original, quat matrix
        position = T[0:3,3]
        [thisPose.pose.position.x, thisPose.pose.position.y, thisPose.pose.position.z] = position
        [thisPose.pose.orientation.x, thisPose.pose.orientation.y, thisPose.pose.orientation.z,thisPose.pose.orientation.w] = quat
        return thisPose

def get_Tmat_from_Pose(self,PoseStamped):  #format
    quat = [PoseStamped.pose.orientation.x, PoseStamped.pose.orientation.y, PoseStamped.pose.orientation.z, PoseStamped.pose.orientation.w]        
    translate = [PoseStamped.pose.position.x, PoseStamped.pose.position.y, PoseStamped.pose.position.z]
    return self.get_Tmat_from_PositionQuat(translate, quat)   # original
    # return translate +quat    

def get_Tmat_from_PositionQuat(self, Position, Quat):    #transformation
    rotationMat = rotation_from_quaternion(Quat)   #original
    T = create_transform_matrix(rotationMat, Position)
    return T

def get_PoseStamped_from_T_initPose(self, T, initPoseStamped):   #transformation
    T_now = self.get_Tmat_from_Pose(initPoseStamped)    #original
    targetPose = self.get_ObjectPoseStamped_from_T(np.matmul(T_now, T))   #original
    return targetPose

def get_Tmat_TranlateInBodyF(self, translate = [0., 0., 0.]): #format
    return create_transform_matrix(np.eye(3), translate)

def get_Tmat_TranlateInZ(self, direction = 1):     #format
    offset = [0.0, 0.0, np.sign(direction)*self.d_z_normal]
    # if step:
    #     offset = [0.0, 0.0, np.sign(direction)*step]
    return self.get_Tmat_TranlateInBodyF(translate = offset)

def get_Tmat_TranlateInY(self, direction = 1):
    offset = [0.0, np.sign(direction)*self.d_lat, 0.0]
    # if step:
    #     offset = [0.0, 0.0, np.sign(direction)*step]
    return self.get_Tmat_TranlateInBodyF(translate = offset)

def get_Tmat_TranlateInX(self, direction = 1):
    offset = [np.sign(direction)*self.d_lat, 0.0, 0.0]
    # if step:
    #     offset = [0.0, 0.0, np.sign(direction)*step]
    return self.get_Tmat_TranlateInBodyF(translate = offset)

def create_transform_matrix(rotation_matrix, translation_vector):
    """
    Creates a homogenous 4x4 matrix representation of this transform

    Parameters
    ----------
    rotation_matrix (3x3 np.ndarray): Rotation between two frames
    translation_vector (3x np.ndarray): Translation between two frames

    """
    return np.r_[np.c_[rotation_matrix, translation_vector],[[0, 0, 0, 1]]]