#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image
from digit_interface import Digit
from cv_bridge import CvBridge

def digitFramePublisher():
    rospy.init_node('digitFramePublisher', anonymous=True)
    pub = rospy.Publisher('digitFrame', Image, queue_size=1)
    
    d = Digit("D20019")
    d.connect()
    print("supported streams: \n {}".format(Digit.STREAMS))
    d.set_resolution(Digit.STREAMS['QVGA'])

    bridge = CvBridge()
    rate = rospy.Rate(30)

    firstframe = None  # Placeholder for the first frame
    show_delta = False  # Flag to control delta display
    amplify = False # flag to control amplification of delta display
    greyscale = False  # Flag to control grayscale display
    LED_on = True  # Flag to control LED state

    while not rospy.is_shutdown():
        try:
            frame = d.get_frame()  # Returns a NumPy array (HxWx3) in RGB
            intensity = frame.mean()  # Calculate the mean intensity of the frame
            image_msg = bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            image_msg.header.stamp = rospy.Time.now()
            image_msg.header.frame_id = "digit_frame"
            pub.publish(image_msg)

            if firstframe is None:
                firstframe = frame.copy()


            # read keyboard
            key = cv2.waitKey(1) & 0xFF
            # q closes digit window
            if key == ord('q'):
                break

            # d toggles delta display
            elif key == ord('d'):
                print("toggling delta display")
                show_delta = not show_delta

            # if c is pressed, save current frame as first frame
            elif key == ord('c'):
                print("zeroing")
                firstframe = frame.copy()

            elif key == ord('a'):
                print("amplifying delta display")
                amplify = not amplify

            # if g is pressed, toggle grayscale display
            elif key == ord('g'):
                print("toggling grayscale display")
                greyscale = not greyscale

            elif key == ord('l'):
                print("LED toggle")
                try:
                    if LED_on:
                        # for i in range(15):
                        #     d.set_intensity(14-i)
                        #     time.sleep(0.05)
                        d.set_intensity(Digit.LIGHTING_MIN)
                    else:
                        d.set_intensity_rgb(15, 15, 15)


                    time.sleep(.2)
                except Exception as e:
                    print(f"Error toggling LED: {e}")
                    continue

                d.disconnect()
                time.sleep(0.2)
                d.connect(LED_intensity=Digit.LIGHTING_MAX if LED_on else Digit.LIGHTING_MIN)


                LED_on = not LED_on

                # burn through next few readings
                for i in range(5):
                    d.get_frame()


            # Show the image in a window
            if show_delta:
                show_frame = cv2.absdiff(frame, firstframe)
            else:
                show_frame = frame.copy()

            # amplify
            if amplify:
                show_frame = cv2.multiply(show_frame, 3)
                show_frame = np.clip(show_frame, 0, 255).astype(np.uint8)

            # convert to grayscale if greyscale is True
            if greyscale:
                show_frame = cv2.cvtColor(show_frame, cv2.COLOR_RGB2GRAY)

            cv2.imshow("DIGIT View", show_frame)

        except rospy.ROSInterruptException:
            break

        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        digitFramePublisher()
    except rospy.ROSInterruptException:
        pass