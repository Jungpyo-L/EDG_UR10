#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped

class FT_CallbackHelp(object):
    def __init__(self):
        rospy.Subscriber("netft_data", WrenchStamped, self.callback_FT)
        
        ## For Force feedback
        self.BufferSize = 7
        self.averagingBuffer =[0.0]*self.BufferSize
        self.inputIndex = 0
        self.startAverage = False
        
        self.averageFx = 0.0
        self.averageFy = 0.0
        self.averageFz = 0.0
        self.averageFx = 0.0
        self.averageFy = 0.0
        self.averageFz = 0.0

        self.offSetFx = 0.0
        self.offSetFy = 0.0
        self.offSetFz = 0.0
        self.offSetTx = 0.0
        self.offSetTy = 0.0
        self.offSetTz = 0.0


        self.thisForce = []
        self.thisFT = []


    def callback_FT(self, data):
                
        thisForce = data.wrench.force
        thisFT = data.wrench
        # self.averagingBuffer[self.inputIndex] = thisForce
        self.averagingBuffer[self.inputIndex] = thisFT
        
        # self.thisForce = thisForce
        self.thisForce = thisFT
        self.inputIndex += 1
        
        if self.inputIndex == len(self.averagingBuffer):
            self.startAverage = True
            self.inputIndex= 0
        if self.startAverage:            
            Fx_sum_dummy = 0.0
            Fy_sum_dummy = 0.0
            Fz_sum_dummy = 0.0
            Tx_sum_dummy = 0.0
            Ty_sum_dummy = 0.0
            Tz_sum_dummy = 0.0
            
            # for force in self.averagingBuffer:
            for wrench in self.averagingBuffer:              
                Fx_sum_dummy += wrench.force.x
                Fy_sum_dummy += wrench.force.y
                Fz_sum_dummy += wrench.force.z
                Tx_sum_dummy += wrench.torque.x
                Ty_sum_dummy += wrench.torque.y
                Tz_sum_dummy += wrench.torque.z

            
            self.averageFx = Fx_sum_dummy / self.BufferSize
            self.averageFy = Fy_sum_dummy / self.BufferSize
            self.averageFz = Fz_sum_dummy / self.BufferSize
            self.averageTx = Tx_sum_dummy / self.BufferSize
            self.averageTy = Ty_sum_dummy / self.BufferSize
            self.averageTz = Tz_sum_dummy / self.BufferSize

            self.averageFx_noOffset = self.averageFx-self.offSetFx
            self.averageFy_noOffset = self.averageFy-self.offSetFy
            self.averageFz_noOffset = self.averageFz-self.offSetFz
            self.averageTx_noOffset = self.averageTx-self.offSetTx
            self.averageTy_noOffset = self.averageTy-self.offSetTy
            self.averageTz_noOffset = self.averageTz-self.offSetTz
    
    def setNowAsBias(self):
        self.offSetFx = self.averageFx
        self.offSetFy = self.averageFy
        self.offSetFz = self.averageFz
        self.offSetTx = self.averageTx
        self.offSetTy = self.averageTy
        self.offSetTz = self.averageTz
    
    # def averageFT_noOffset(self):

                  