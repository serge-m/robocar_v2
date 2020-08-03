#!/usr/bin/env python

import math
import numpy
import threading

from math import pi

import rospy
import tf

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers


def value_generator():
    while True:
        for i in range(0, 100, 1):
            yield 4 * pi * i / 100.

        for i in range(100, 0, -1):
            yield 4 * pi * i / 100.

class WheelController(object):
    def __init__(self):
        rospy.init_node("wheel_controller")
        list_ctrlrs = rospy.ServiceProxy("controller_manager/list_controllers",
                                         ListControllers)
        list_ctrlrs.wait_for_service()
        self._sleep_timer = rospy.Rate(3)
        self.axle_pub = create_command_publisher(list_ctrlrs, "axle_ctrlr")
        
        
    def spin(self):
        values = value_generator()
        while not rospy.is_shutdown():
            val = next(values)    
            print("wheel_controller: publishing position {}".format(val))
            self.axle_pub.publish(val)
            self._sleep_timer.sleep()


def wait_for_controller(list_ctrlrs, ctrlr_name):
    while True:
        response = list_ctrlrs()
        for ctrlr in response.controller:
            if ctrlr.name == ctrlr_name:
                if ctrlr.state == "running":
                    return
                rospy.sleep(0.1)
                break



def create_command_publisher(list_ctrlrs, ctrlr_name):
    wait_for_controller(list_ctrlrs, ctrlr_name)
    return rospy.Publisher(ctrlr_name + "/command", Float64, queue_size=1)

if __name__ == "__main__":
    ctrlr = WheelController()
    ctrlr.spin()
