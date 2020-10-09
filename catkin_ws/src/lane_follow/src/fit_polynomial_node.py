#!/usr/bin/env python
# Node that takes in warped tresholded camera image 
# detects lane lines and fit polynomials to them
from __future__ import print_function, division

import sys
import numpy as np
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

from img_helpers import FitPolynomial

class FitPolynomialNode:
    def __init__(self):
        

        self.FitPolynomial = FitPolynomial()

        self.bridge = CvBridge()

        self.subscriber = rospy.Subscriber("/top_view", Image, self.imageCallback, queue_size=1)
        self.publisher = rospy.Publisher ("/camera/color/polynomial", String, queue_size=1)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
          
            if (self.FitPolynomial.left_fit is not None) and (self.FitPolynomial.right_fit is not None):
                msg = str(self.FitPolynomial.left_fit) + ", " + str(self.FitPolynomial.right_fit)
                self.publisher.publish(msg)
                rate.sleep()

    def imageCallback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            self.FitPolynomial.fit_polynomial(cv_image)         
            
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('FitPolynomial_node', anonymous=True)
    node = FitPolynomialNode()

    try:
        print("running Fitting Polynomial ")
    except KeyboardInterrupt:
        print("Shutting down ROS FitPolynomial module")
	

if __name__ == '__main__':
    main(sys.argv)  

	
