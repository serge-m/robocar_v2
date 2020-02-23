#!/usr/bin/env python3
"""
Lane following module
"""
from __future__ import print_function, division

import sys, time
import numpy as np
import cv2
import roslib
import rospy

from sensor_msgs.msg import CompressedImage
from lf.lane_follower import LaneFollower


class ImageSubPubNode:
    def __init__(self, processor: callable):
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback,  queue_size=1)
        self.publisher = rospy.Publisher("/lane_follow/debug/compressed", CompressedImage, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        self.processor = processor
    
    def callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        result = self.processor(image_np)
        msg = self.compressed_img_message(result)
        self.publisher.publish(msg)

    def compressed_img_message(self, image_np):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        retval, encoded = cv2.imencode('.jpg', image_np)
        if not retval:
            raise RuntimeError("Unable to encode image, {}".format(image_np))
        msg.data = np.array(encoded).tostring()
        return msg

    def shutdown(self):
        print("ImageSubPubNode shut down")


def main(args):
    rospy.init_node('lane_follow', anonymous=True)
    lane_follower = LaneFollower()
    node = ImageSubPubNode(processor=lane_follower)
    
    try:
        print("running ImageSubPubNode")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")

if __name__ == '__main__':
    main(sys.argv)