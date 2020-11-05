#!/usr/bin/env python
# Node that takes in warped tresholded camera image 
# detects lane lines and fit polynomials to them
from __future__ import print_function, division

import sys
import numpy as np
import rospy
import tf

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from robocar_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import Quaternion

from img_helpers import FitPolynomial

class FitPolynomialNode:
    def __init__(self):
        self.velocity = rospy.get_param('~velocity')

        self.FitPolynomial = FitPolynomial()

        self.bridge = CvBridge()

        self.subscriber = rospy.Subscriber("/top_view", Image, self.imageCallback, queue_size=1)
        self.publisher = rospy.Publisher ("/waypoints/update", Lane, queue_size=1)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
          
            if (self.FitPolynomial.left_fit is not None) and (self.FitPolynomial.right_fit is not None):
                waypoints = self.FitPolynomial.get_waypoints()
                self.publisher.publish(self.makeLane(waypoints))
                rate.sleep()

    def imageCallback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            self.FitPolynomial.fit_polynomial(cv_image)         
            
        except CvBridgeError as e:
            print(e)

    def makeLane(self, waypoints):
        msg = []

        for i in range(len(waypoints)):
            p = Waypoint()
            p.pose.pose.position.x = float(waypoints[i][0])
            p.pose.pose.position.y = float(waypoints[i][1])
            p.pose.pose.position.z = float(0)
            q = tf.transformations.quaternion_from_euler(0., 0., 0)
            p.pose.pose.orientation = Quaternion(*q)
            p.twist.twist.linear.x = float(self.velocity)

            msg.append(p) 

        lane = Lane()
        lane.header.frame_id = 'base_link'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = msg

        return lane


def main(args):
    rospy.init_node('FitPolynomial_node', anonymous=True)
    node = FitPolynomialNode()

    try:
        print("running Fitting Polynomial ")
    except KeyboardInterrupt:
        print("Shutting down ROS FitPolynomial module")
	

if __name__ == '__main__':
    main(sys.argv)  

	
