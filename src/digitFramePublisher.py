#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from digit_interface import Digit
from cv_bridge import CvBridge

def digitFramePublisher():
    rospy.init_node('digitFramePublisher', anonymous=True)
    pub = rospy.Publisher('digitFrame', Image, queue_size=1)
    
    d = Digit("D20019")
    d.connect()

    bridge = CvBridge()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        try:
            frame = d.get_frame()  # Returns a NumPy array (HxWx3) in RGB
            image_msg = bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            image_msg.header.stamp = rospy.Time.now()
            image_msg.header.frame_id = "digit_frame"
            pub.publish(image_msg)

            # Show the image in a window
            cv2.imshow("DIGIT View", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except rospy.ROSInterruptException:
            break

        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        digitFramePublisher()
    except rospy.ROSInterruptException:
        pass