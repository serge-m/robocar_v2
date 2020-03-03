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
    pub = rospy.Publisher("pwm_radio_arduino/driver_ackermann", AckermannDrive, queue_size=1)
    pub_tfm = rospy.Publisher("pwm_radio_arduino/steering_tfm", steering_tfm, queue_size=1)
    rospy.init_node('driver', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        tfm = steering_tfm(-1, 0, 1, 1000, 1400, 1800, -1, 0, 1, 1000, 1400, 1800)
        pub_tfm.publish(tfm)
        for angle in np.arange(-1, 1, 0.1):
            msg = AckermannDrive(steering_angle=angle)
            pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

