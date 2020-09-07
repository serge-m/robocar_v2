#!/usr/bin/env python
"""
Lane following module
"""
from __future__ import print_function, division

import sys, time
import numpy as np
import cv2
import roslib
import rospy


from geometry_msgs.msg import Vector3Stamped, Vector3
from sensor_msgs.msg import CompressedImage

from lane_follower import LaneFollower
from img_messages import compressed_img_message, np_from_compressed_ros_msg

# (vector: np.ndarray) -> Vector3Stamped
def vector3d_msg(vector):
    msg = Vector3Stamped()
    msg.header.stamp=rospy.Time.now()
    msg.vector=Vector3(vector[0], vector[1], 0)
    return msg

class ImageSubPubNode:
    def __init__(self, processor):
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback,  queue_size=1)
        self.publisher_debug = rospy.Publisher("/lane_follow/debug/compressed", CompressedImage, queue_size=1)
        self.publisher_shift = rospy.Publisher("/lane_follow/desired_shift", Vector3Stamped, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        self.processor = processor
    
    def callback(self, ros_data):
        image_np = np_from_compressed_ros_msg(ros_data)
        debug_output = {}
        desired_shift = self.processor(image_np, debug_output)
        self.publisher_shift.publish(vector3d_msg(desired_shift))
        if 'vis' in debug_output:
            self.publisher_debug.publish(compressed_img_message(debug_output['vis']))
        

    def shutdown(self):
        print("ImageSubPubNode shut down")


def main(args):
    rospy.init_node('lane_follow', anonymous=True)
    camera_params = rospy.get_param('~camera_params')
    
    lane_follower = LaneFollower(camera_params, resize_if_needed=True)
    node = ImageSubPubNode(processor=lane_follower)
    
    try:
        print("running ImageSubPubNode, camera_params {}".format(camera_params))
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")

if __name__ == '__main__':
    main(sys.argv)   