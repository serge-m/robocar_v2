#!/usr/bin/env python
# Node that takes in camera image 
# and with perspective transformation 
# makes an image from birds eye perspective
from __future__ import print_function, division

import sys
import numpy as np
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from img_helpers import BirdsEye

class BirdsEyeNode:
    def __init__(self):
        # TODO make all parametrised from camera info, tf, etc
        # Camera calibration parameters (in pixels)
        fx = 742.64941
        fy = 742.64941
        cx = 666.99677
        cy = 517.3488

        # Camera position (in metres, degrees)
        H = 0.21
        theta = 0.00
        # theta *= (M_PI/180.0)

        # Defining desired field-of-view (in metres)
        # Ox and Oy are the position of the projection of the optical center on the 
        # ground plane from (0, 0) of the top-view image
        # Wx and Wy are the width and height of the top-view image
        # x-axis is the direction pointing right in the top-view image
        # y-axis is the direction pointing down in the top-view image
        Wx = 4.8
        Wy = 3.6
        Ox = Wx / 2
        Oy = Wy

        # Scaling factor (in pixels/m)
        # (Use 80.0 for realtime low-res output but 160.0 for datasets)
        s = 100.0

        self.birdsEyeImage = BirdsEye((fx, fy), (cx, cy), H, theta, (Ox, Oy), (Wx, Wy), s)

        self.bridge = CvBridge()

        self.subscriber = rospy.Subscriber("/camera/color/image_rect_color", Image, self.imageCallback, queue_size=1)
        self.publisher = rospy.Publisher ("/top_view", Image, queue_size=1)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if (np.count_nonzero(self.birdsEyeImage.birds_image) > 0):
                top_view_msg = self.bridge.cv2_to_imgmsg(self.birdsEyeImage.birds_image)
                self.publisher.publish(top_view_msg)
                rate.sleep()

    def imageCallback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            self.birdsEyeImage.warpPerspective(cv_image)         
            
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('birds_eye_node', anonymous=True)
    node = BirdsEyeNode()

    try:
        print("running birdsEye transformation")
    except KeyboardInterrupt:
        print("Shutting down ROS birdsEye transformation module")
	

if __name__ == '__main__':
    main(sys.argv)  

	
