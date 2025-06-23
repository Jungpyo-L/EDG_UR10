#!/usr/bin/env python

import os
import datetime
import numpy as np
import re
from geometry_msgs.msg import PoseStamped
from helperFunction.utils import rotation_from_quaternion, create_transform_matrix, rotationFromQuaternion, normalize, hat, quaternionFromMatrix, quaternion_from_matrix
from scipy.spatial.transform import Rotation as Rot
import scipy
from icecream import ic


class adaptMotionHelp(object):
    def __init__(self, d_w=15, d_lat = 5e-3, d_z= 1.5e-3):   #original
        self.dw = d_w * np.pi / 180.0
        self.d_lat = d_lat
        self.d_z_normal = d_z


    def get_ObjectPoseStamped_from_T(self,T):   #transformation
        thisPose = PoseStamped()
        thisPose.header.frame_id = "base_link"
        R = T[0:3,0:3]
        quat = quaternion_from_matrix(R)
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
        return self.get_ObjectPoseStamped_from_T(np.matmul(T_now, T))   # gsa * gab = gsb

    def get_Tmat_TranlateInBodyF(self, translate = [0., 0., 0.]): #format
        return create_transform_matrix(np.eye(3), translate)

    def get_Tmat_TranslateInZ(self, direction = 1):     #format
        offset = [0.0, 0.0, np.sign(direction)*self.d_z_normal]
        # if step:
        #     offset = [0.0, 0.0, np.sign(direction)*step]
        return self.get_Tmat_TranlateInBodyF(translate = offset)

    def get_Tmat_TranslateInY(self, direction = 1):
        offset = [0.0, np.sign(direction)*self.d_lat, 0.0]
        # if step:
        #     offset = [0.0, 0.0, np.sign(direction)*step]
        return self.get_Tmat_TranlateInBodyF(translate = offset)
    
    def get_Tmat_TranslateInX(self, direction = 1):
        offset = [np.sign(direction)*self.d_lat, 0.0, 0.0]
        # if step:
        #     offset = [0.0, 0.0, np.sign(direction)*step]
        return self.get_Tmat_TranlateInBodyF(translate = offset)
    
    def get_Tmat_RotateInX(self, direction = 1):
        rot_axis = np.array([np.sign(direction), 0.0, 0.0])
        omega_hat = hat(rot_axis)
        Rw = scipy.linalg.expm(self.dw* omega_hat)
        return create_transform_matrix(Rw, [0,0,0])
    
    def get_Tmat_RotateInY(self, direction = 1):
        rot_axis = np.array([0.0, np.sign(direction), 0.0])
        omega_hat = hat(rot_axis)
        Rw = scipy.linalg.expm(self.dw* omega_hat)
        return create_transform_matrix(Rw, [0,0,0])
    
    def get_Tmat_RotateInZ(self, direction = 1):
        rot_axis = np.array([0.0, 0.0, np.sign(direction)])
        omega_hat = hat(rot_axis)
        Rw = scipy.linalg.expm(self.dw* omega_hat)
        return create_transform_matrix(Rw, [0,0,0])
    
    def get_Tmat_axialMove(self, F_normal, F_normalThres):
        
        if F_normal < -F_normalThres:
            # move upward
            T_normalMove = self.get_Tmat_TranlateInZ(direction = -1)
        elif F_normal > F_normalThres:
            # move downward
            T_normalMove = self.get_Tmat_TranlateInZ(direction = 1)
        else:
            T_normalMove = np.eye(4)
        return T_normalMove