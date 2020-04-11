#!/usr/bin/env python
"""
Mouse odometry
"""
from __future__ import print_function, division

import sys, time
import rospy
import math
import threading
import struct
from nav_msgs.msg import Odometry


pos = {
    'x': 0.,
    'y': 0.,
    'theta': 0.,
    'dx': 0.,
    'dy': 0.,
    'dtheta': 0.
}


def start_thread():
    thread = threading.Thread(target=read_mouse, args=())
    thread.setDaemon(True)
    thread.start()

def read_mouse():
    with open("/dev/input/mice", "rb") as f:
        while not rospy.is_shutdown():
            data = f.read(3);
            _, dy, dx = struct.unpack("bbb", data)
            update_pos(dx, dy)
            

def update_pos(dx, dy):
    dx = dx / 10000.
    dy = dy / 10000.
    dtheta = math.atan2(dy, dx)
    pos['dx'] = dx
    pos['dy'] = dy
    pos['dtheta'] = dtheta
    pos['theta'] += dtheta
    pos['x'] += dx
    pos['y'] += dy
    # rospy.loginfo("new dpose {} {}".format(dx, dy))
     

def main_mouse_odometery():
    rospy.loginfo("main_mouse_odometery started")
    pub = rospy.Publisher("robocar/odometry", Odometry, queue_size=1, latch=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = make_odom_message(pos, rospy.Time.now())
        pub.publish(msg)
        rate.sleep()
        rospy.loginfo_throttle(1, "last published pos {} {}".format(pos['x'], pos['y']))
        
    rospy.loginfo("main_mouse_odometery exited")

def make_odom_message(pos, ros_now):
    """
    TODO: compute properly
    """
    
    odometry = Odometry()

    odometry.header.frame_id = "odom"
    odometry.header.stamp = ros_now
    odometry.pose.pose.position.x = pos['x']
    odometry.pose.pose.position.y = pos['y']
    odometry.pose.pose.position.z = 0
    
    odometry.child_frame_id = "base_link"
    odometry.twist.twist.linear.x = pos['dx']
    odometry.twist.twist.linear.y = pos['dy']
    odometry.twist.twist.angular.z = pos['dtheta']
    # rospy.loginfo("pos {}".format(pos))
    return odometry

if __name__ == '__main__':
    try:
        rospy.init_node('mouse_odometry', anonymous=True)
        start_thread()
        main_mouse_odometery()
    except rospy.ROSInterruptException:
        print("interrupted by ros")

