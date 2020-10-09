#!/usr/bin/env python
# Node that takes in camera image 
# and tresholds it
from __future__ import print_function, division

import sys
import numpy as np
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from img_helpers import Treshold

class TresholdNode:
    def __init__(self):
        

        self.TresholdImage = Treshold()

        self.bridge = CvBridge()

        self.subscriber = rospy.Subscriber("/camera/color/image_rect_color", Image, self.imageCallback, queue_size=1)
        self.publisher = rospy.Publisher ("/camera/color/tresholded_image", Image, queue_size=1)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if (np.count_nonzero(self.TresholdImage.image) > 0):
                treshold_msg = self.bridge.cv2_to_imgmsg(self.TresholdImage.image)
                self.publisher.publish(treshold_msg)
                rate.sleep()

    def imageCallback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            self.TresholdImage.treshold_binary(cv_image)         
            
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('treshold_node', anonymous=True)
    node = TresholdNode()

    try:
        print("running binary tresholding ")
    except KeyboardInterrupt:
        print("Shutting down ROS binary tresholding module")
	

if __name__ == '__main__':
    main(sys.argv)  

	
