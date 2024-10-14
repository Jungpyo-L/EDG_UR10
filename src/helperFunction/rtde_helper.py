#!/usr/bin/env python
try:
  import rospy
  import tf
  ros_enabled = True
except:
  print('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
  ros_enabled = False

from grp import getgrall
from hmac import trans_36
import numpy as np
from geometry_msgs.msg import PoseStamped

from .transformation_matrix import *
from .utils import *

import rtde_control
import rtde_receive

from scipy.spatial.transform import Rotation as R
import copy
import numpy as np



class rtdeHelp(object):
    def __init__(self, rtde_frequency = 125, speed = 0.3, acc = 0.2):
        self.tfListener = tf.TransformListener()
        self.rtde_frequency = rtde_frequency

        self.rtde_c = rtde_control.RTDEControlInterface("10.0.0.1", rtde_frequency)
        self.rtde_r = rtde_receive.RTDEReceiveInterface("10.0.0.1", rtde_frequency)

        self.checkDistThres = 1e-3
        self.checkQuatThres = 10e-3
        self.speed = speed
        self.acc = acc

    def _append_ns(self, in_ns, suffix):
        """
        Append a sub-namespace (suffix) to the input namespace
        @param in_ns Input namespace
        @type in_ns str
        @return Suffix namespace
        @rtype str
        """
        ns = in_ns
        if ns[-1] != '/':
            ns += '/'
        ns += suffix
        return ns

    def getPoseObj(self, goalPosition, setOrientation):
        Pose = PoseStamped()  
        
        Pose.header.frame_id = "base_link"
        Pose.header.stamp = rospy.Time.now()
        Pose.pose.orientation.x = setOrientation[0]
        Pose.pose.orientation.y = setOrientation[1]
        Pose.pose.orientation.z = setOrientation[2]
        Pose.pose.orientation.w = setOrientation[3]
        
        Pose.pose.position.x = goalPosition[0]
        Pose.pose.position.y = goalPosition[1]
        Pose.pose.position.z = goalPosition[2]
        
        return Pose

    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2,
                w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2)
    
    def getRotVector(self, goalPose):
        qx = goalPose.pose.orientation.x
        qy = goalPose.pose.orientation.y
        qz = goalPose.pose.orientation.z
        qw = goalPose.pose.orientation.w
        r = R.from_quat([qx, qy, qz, qw])
        Rx, Ry, Rz = r.as_rotvec()
        return Rx, Ry, Rz

    def getTCPPose(self, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        Rx, Ry, Rz = self.getRotVector(pose)
        return [x, y, z, Rx, Ry, Rz]
    
    def speedl(self, goalPose,speed=0.5, acc=0.5, time=0.5, aRot='a'):

        if len(goalPose) != 6:
            raise ValueError("Target pose must have 6 elements: [x, y, z, Rx, Ry, Rz]")
        try:
        # Perform linear motion using moveL function
            self.rtde_c.speedL(goalPose, self.speed, self.acc, time, aRot)
        except Exception as e:
            print(f"Error occurred during linear motion: {e}")

    
    def setPayload(self, payload, CoG):
        # Assuming method is avalible within RTDEControlInterface
        self.rtde_c.set_payload(payload, CoG)

    def goToPositionOrientation(self, goalPosition, setOrientation, asynchronous = False):
        self.goToPose(getPoseObj(goalPosition, setOrientation))

    def goToPose(self, goalPose, speed = 0.1, acc = 0.1, asynchronous=False):
        targetPose = self.getTCPPose(goalPose)
        self.rtde_c.moveL(targetPose, self.speed, self.acc, asynchronous)

    def goToPoseAdaptive(self, goalPose, speed = 0.0, acc = 0.0,  time = 0.05, lookahead_time = 0.2, gain = 100.0):         # normal force measurement
        t_start = self.rtde_c.initPeriod()
        targetPose = self.getTCPPose(goalPose)
        self.rtde_c.servoL(targetPose, speed, acc, time, lookahead_time, gain)
        self.rtde_c.waitPeriod(t_start)

    def checkGoalPoseReached(self, goalPose, checkDistThres=np.nan, checkQuatThres = np.nan):
        if np.isnan(checkDistThres):
            checkDistThres=self.checkDistThres
        if np.isnan(checkQuatThres):
            checkQuatThres = self.checkQuatThres
        (trans1,rot) = self.tfListener.lookupTransform('/base_link', '/tool0', rospy.Time(0))          
        goalQuat = np.array([goalPose.pose.orientation.x,goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w])
        rot_array = np.array(rot)
        quatDiff = np.min([np.max(np.abs(goalQuat - rot_array)), np.max(np.abs(goalQuat + rot_array))])
        distDiff = np.linalg.norm(np.array([goalPose.pose.position.x,goalPose.pose.position.y, goalPose.pose.position.z])- np.array(trans1)) 
        # print(quatDiff, distDiff)
        print("quatdiff: %.4f" % quatDiff)
        print("distDiff: %.4f" % distDiff)
        return distDiff < checkDistThres and quatDiff < checkQuatThres

        
    def readCurrPositionQuat(self):
        (trans1,rot) = self.tfListener.lookupTransform('/base_link', '/tool0', rospy.Time(0))         
        return (trans1, rot) #trans1= position x,y,z, // quaternion: x,y,z,w

    def stopAtCurrPose(self,asynchronous = True):
        currPosition, orientation = self.readCurrPositionQuat()

        # always false
        # wait = False
        self.goToPositionOrientation(currPosition, orientation, asynchronous=asynchronous)
    
    def stopAtCurrPoseAdaptive(self):
        self.rtde_c.servoStop()
        # self.rtde_c.stopScript()

    # Get current pose from TF
    def getCurrentPoseTF(self):
        (Position, Orientation) = self.readCurrPositionQuat()
        return getPoseObj(Position, Orientation)

    def getCurrentPose(self):
        TCPPose = self.rtde_r.getActualTCPPose()
        Position = [TCPPose[0], TCPPose[1], TCPPose[2]]
        r = R.from_rotvec(np.array([TCPPose[3], TCPPose[4], TCPPose[5]]))
        return getPoseObj(Position, r.as_quat())

    def getTCPoffset(self):
        return self.rtde_c.getTCPOffset()

    def setTCPoffset(self, offset):
        return self.rtde_c.setTcp(offset)

    def getMethodsName_r(self):
        object_methods = [method_name for method_name in dir(self.rtde_r) if callable(getattr(self.rtde_r, method_name))]
        print(object_methods)

    def getMethodsName_c(self):
        object_methods = [method_name for method_name in dir(self.rtde_c) if callable(getattr(self.rtde_c, method_name))]
        print(object_methods)