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
import threading


def start_thread_pub_tfm_params():
    thread = threading.Thread(target=pub_tfm_params,args=())
    thread.setDaemon(True)
    thread.start()

def pub_tfm_params():
    pub_tfm = rospy.Publisher("pwm_radio_arduino/steering_tfm", steering_tfm, queue_size=1, latch=False)
    while not rospy.is_shutdown():
        tfm = steering_tfm(-1, 0, 1, 90-30, 90, 90+30, -1, 0, 1, 90-10, 90, 90+10)
        pub_tfm.publish(tfm)  
        rospy.loginfo("pub_tfm")
        time.sleep(1) 

def talker():
    pub = rospy.Publisher("pwm_radio_arduino/driver_ackermann", AckermannDrive, queue_size=1, latch=False)
    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
        for angle in np.hstack([np.arange(-1, 1, 0.1), np.arange(1, -1, -0.1)]):
            msg = AckermannDrive(speed=0, steering_angle=angle)
            pub.publish(msg)
            rospy.loginfo("pub_drive")
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('driver', anonymous=True)
        start_thread_pub_tfm_params()
        talker()
    except rospy.ROSInterruptException:
        pass

