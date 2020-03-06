#!/usr/bin/env python3
"""
Main driver node
"""
from __future__ import print_function, division

import sys, time
import numpy as np
import cv2
import roslib
import rospy

from ackermann_msgs.msg import AckermannDrive
from pwm_radio_arduino.msg import steering_tfm

def talker():
    pub = rospy.Publisher("pwm_radio_arduino/driver_ackermann", AckermannDrive, queue_size=1, latch=False)
    pub_tfm = rospy.Publisher("pwm_radio_arduino/steering_tfm", steering_tfm, queue_size=1, latch=False)
    rospy.init_node('driver', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
        for angle in np.hstack([np.arange(-1, 1, 0.1), np.arange(1, -1, -0.1)]):
            msg = AckermannDrive(speed=0, steering_angle=angle)
            pub.publish(msg)
            tfm = steering_tfm(-1, 0, 1, 1200, 1400, 1600, -1, 0, 1, 1300, 1400, 1500)
            pub_tfm.publish(tfm)    
            print("1")    
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

