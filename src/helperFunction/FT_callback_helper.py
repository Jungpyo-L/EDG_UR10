#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped

class FT_CallbackHelp(object):
    def __init__(self):
        
        # Subscriber to the raw sensor data
        rospy.Subscriber("netft_data", WrenchStamped, self.callback_FT)

        # Publisher for the offset (bias-subtracted) data
        self.pub_offsetted = rospy.Publisher("netft_data_offset", 
                                             WrenchStamped, 
                                             queue_size=10)
        
        ## For Force feedback
        self.BufferSize = 7
        self.averagingBuffer =[0.0]*self.BufferSize
        self.inputIndex = 0
        self.startAverage = False
        
        self.averageFx = 0.0
        self.averageFy = 0.0
        self.averageFz = 0.0
        self.averageTx = 0.0
        self.averageTy = 0.0
        self.averageTz = 0.0

        self.offSetFx = 0.0
        self.offSetFy = 0.0
        self.offSetFz = 0.0
        self.offSetTx = 0.0
        self.offSetTy = 0.0
        self.offSetTz = 0.0


        self.thisForce = []


    def callback_FT(self, data):

        """
        Callback to process the incoming WrenchStamped from netft_data.
        We store them in a circular buffer of length BufferSize,
        compute an average, and publish with offset subtracted.
        """
                
        # Store the newest WrenchStamped in the buffer
        self.averagingBuffer[self.inputIndex] = data.wrench
        self.thisForce = data.wrench
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

            # Now publish the offset WrenchStamped
            offset_wrench_msg = WrenchStamped()
            offset_wrench_msg.header.stamp = rospy.Time.now()
            offset_wrench_msg.header.frame_id = data.header.frame_id

            offset_wrench_msg.wrench.force.x = self.averageFx_noOffset
            offset_wrench_msg.wrench.force.y = self.averageFy_noOffset
            offset_wrench_msg.wrench.force.z = self.averageFz_noOffset
            offset_wrench_msg.wrench.torque.x = self.averageTx_noOffset
            offset_wrench_msg.wrench.torque.y = self.averageTy_noOffset
            offset_wrench_msg.wrench.torque.z = self.averageTz_noOffset

            self.pub_offsetted.publish(offset_wrench_msg)
    
    def setNowAsBias(self):
        """
        Call this to treat the current average as bias.
        E.g. you might call this via a ROS service or 
        a keypress in a separate script.
        """
        self.offSetFx = self.averageFx
        self.offSetFy = self.averageFy
        self.offSetFz = self.averageFz
        self.offSetTx = self.averageTx
        self.offSetTy = self.averageTy
        self.offSetTz = self.averageTz
    
    # def averageFT_noOffset(self):
if __name__ == "__main__":
    rospy.init_node("offset_ft_publisher")
    ft_callback_helper = FT_CallbackHelp()
    rospy.spin()

                  