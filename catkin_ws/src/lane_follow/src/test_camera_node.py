#!/usr/bin/env python
"""
Test Camera module
"""
from __future__ import print_function, division

import sys
import rospy
import tf
import numpy as np
from sympy import Point3D, Line3D, Plane

from cv_bridge import CvBridge, CvBridgeError
from camera_info_manager import *
from image_geometry import PinholeCameraModel

from geometry_msgs.msg import Quaternion, PoseStamped, Point
from sensor_msgs.msg import Image, CameraInfo

from visualization_msgs.msg import Marker

class TestCameraNode:
    def __init__(self):
        self.camera_model = PinholeCameraModel()
        self.bridge = CvBridge()
        # load sensor_msgs/CameraInfo Message from yaml file if it exists
        url = "package://robocar_simulation/config/sim_camera_calibration.yaml"
        cim = camera_info_manager.CameraInfoManager(cname='camera', url=url)
        cim.loadCameraInfo()
        camera_info_msg = cim.getCameraInfo()
        # load parameters from sensor_msgs/CameraInfo Message to Camera Model
        self.camera_model.fromCameraInfo(camera_info_msg)
        print("vectors which go through camera center and image pixel")
        # left bottom corner        
        lbc_ray = self.camera_model.projectPixelTo3dRay((0, 719))
        print(lbc_ray)
        # right bottom corner        
        rbc_ray = self.camera_model.projectPixelTo3dRay((1279, 719))
        print(rbc_ray)        
        #  now these vectors should be transformed to the base_link frame
        self.tf_listener = tf.TransformListener()
        self.waypoint_pub = rospy.Publisher('/visualize/points', Marker, queue_size=1)
        while not rospy.is_shutdown():
            try:
                # camera center in the base_link frame
                zero = self.transformPoint((0,0,0), "camera_link_optical", "base_link")
                # bottom points
                lbc_point = self.transformPoint(lbc_ray, "camera_link_optical", "base_link")
                rbc_point = self.transformPoint(rbc_ray, "camera_link_optical", "base_link")
                # get lines from camera center and ray in the base_link frame
                lbc_line = Line3D(Point3D(zero), Point3D(lbc_point))
                rbc_line = Line3D(Point3D(zero), Point3D(rbc_point))
                # TODO get -0.092 from tf or somewhere else
                # ground plane in the base_link frame
                xy_plane = Plane(Point3D(0, 0, -0.092), normal_vector=(0, 0, 1))
                # bottom points in the base_link frame
                # are intersection points of lines and ground plane
                point1 = xy_plane.intersection(lbc_line)[0]
                point2 = xy_plane.intersection(rbc_line)[0]
                print("bottom points")
                print(float(point1.x), float(point1.y), float(point1.z))
                print(float(point2.x), float(point2.y), float(point2.z))
                # distance in meters between points
                print(float(point1.distance(point2)))
                # translate factor depends on h*w of the image
                t = 720*float(point1.distance(point2))/1280
                print("translate factor in meters ", t)
                # upper points in the base_link frame
                point3 = point1.translate(t)
                point4 = point2.translate(t)
                print("upper points in base_link frame")
                print(float(point3.x), float(point3.y), float(point3.z))
                print(float(point4.x), float(point4.y), float(point4.z))
                # transform points 3 and 4 to camera_optical_link frame
                luc_point = self.transformPoint(point3, "base_link", "camera_link_optical")
                ruc_point = self.transformPoint(point4, "base_link", "camera_link_optical")
                print("upper points in optical frame")
                print(luc_point)
                print(ruc_point)
                # pixel coordinates of these points:
                print("upper points in pixel")
                luc = self.camera_model.project3dToPixel(luc_point)
                print(luc)
                # right bottom corner        
                ruc = self.camera_model.project3dToPixel(ruc_point)
                print(ruc)                        
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def transformPoint(self, point, source_frame, dest_frame):
        # make geometry_msgs/PoseStamped Message
        p = PoseStamped()
        p.pose.position.x = point[0]
        p.pose.position.y = point[1]
        p.pose.position.z = point[2]
        q = tf.transformations.quaternion_from_euler(0., 0., 0)
        p.pose.orientation = Quaternion(*q)   
        p.header.frame_id = source_frame
        p.header.stamp = rospy.Time()           
        
        # apply transformation to a pose between source_frame and dest_frame
        new_point = self.tf_listener.transformPose(dest_frame, p)
        p = new_point.pose.position
        
        return p.x, p.y, p.z
               
def main(args):
    rospy.init_node('test_camera_node', anonymous=True)
    node = TestCameraNode()

    try:
        print("running Test Camera Node")
    except KeyboardInterrupt:
        print("Shutting down ROS Test Camera module")

if __name__ == '__main__':
    main(sys.argv)  
