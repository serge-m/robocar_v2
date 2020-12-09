#!/usr/bin/env python
"""
Lane following module
"""
from __future__ import print_function, division

import sys
import rospy
import tf

from cv_bridge import CvBridge, CvBridgeError
from camera_info_manager import *
from image_geometry import PinholeCameraModel

from geometry_msgs.msg import Quaternion, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from robocar_msgs.msg import Lane, Waypoint

from lane_follower import LaneFollower
from img_helpers import getUpperPoints

class LaneFollowNode:
    def __init__(self):
        self.lane_follower = LaneFollower()
        self.camera_model = PinholeCameraModel()
        self.bridge = CvBridge()
        self.velocity = rospy.get_param('~velocity')
        self.cameraInfoSet = False
        self.hasTransformMatrix = False
        self.canTransformImg = False

        # load sensor_msgs/CameraInfo Message from yaml file if it exists
        file_camera_params = rospy.get_param('~camera_params')
        camera_info_msg = camera_info_manager.loadCalibrationFile(file_camera_params, cname='camera')
        # if there is no yaml file msg is filled with zeros, no error msg
        if (camera_info_msg.width != 0):              
            self.info_cb(camera_info_msg)
        else:
            # get all parameters from sensor_msgs/CameraInfo Message
            self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_cb,  queue_size=1)
                        
        # get camera position (in metres, degrees) from tf topic
        self.tf_listener = tf.TransformListener()       
        
        # image after rectification, no need to undistort
        self.img_sub = rospy.Subscriber("/camera/color/image_rect_color", Image, self.img_cb,  queue_size=1)
        
        # publish waypoints of a lane
        self.publisher = rospy.Publisher ("/waypoints/update", Lane, queue_size=1)
        self.img_pub = rospy.Publisher ("/camera/color/top_view", Image, queue_size=1)
        
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown(): 
            # camera model is loaded but TransformMatrix is not ready
            if self.cameraInfoSet and (not self.hasTransformMatrix):
                try:
                    # find src and dst points for 
                    # LaneFollower.image_proc.get_transform_matrix(src, dst)
                    # it is done here because points 
                    # should be transformed between frames with tfTransformer
                    h = self.camera_model.height
                    w = self.camera_model.width
                    # left bottom corner        
                    lbc_ray = self.camera_model.projectPixelTo3dRay((0, h))
                    # right bottom corner        
                    rbc_ray = self.camera_model.projectPixelTo3dRay((w, h))
                    # camera center and bottom points in the base_bottom_link frame
                    camera_link_optical = rospy.get_param('~camera_opt_frame')
                    base_bottom_link = rospy.get_param('~base_frame')
                    zero = self.get_point_xyz(self.transformPoint((0,0,0), camera_link_optical, base_bottom_link))
                    lbc_point = self.get_point_xyz(self.transformPoint(lbc_ray, camera_link_optical, base_bottom_link))
                    rbc_point = self.get_point_xyz(self.transformPoint(rbc_ray, camera_link_optical, base_bottom_link))
                    # get scale parameter from params server
                    y_scale = rospy.get_param('~y_scale')
                    point3, point4, x_scale = getUpperPoints(zero, lbc_point, rbc_point, y_scale)
                    # transform points 3 and 4 to camera_optical_link frame
                    luc_point = self.get_point_xyz(self.transformPoint(point3, base_bottom_link, camera_link_optical))
                    ruc_point = self.get_point_xyz(self.transformPoint(point4, base_bottom_link, camera_link_optical))
                    # pixel coordinates of these points:
                    # left upper corner 
                    luc = self.camera_model.project3dToPixel(luc_point)
                    # right upper corner        
                    ruc = self.camera_model.project3dToPixel(ruc_point)
                    # form src and dst points
                    src = [[luc[0], luc[1]],[ruc[0], ruc[1]],[w, h],[0, h]]
                    dst = [[0, 0],[w, 0],[w, h],[0, h]]
                    # get matrix for perspective transformation
                    if (src and dst):
                        self.lane_follower.image_proc.get_transform_matrix(src, dst)
                        self.lane_follower.image_proc.setScale((x_scale/w, y_scale/h))
                        self.hasTransformMatrix = True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                
            if self.canTransformImg: 
                try:
                    # publish warped tresholded image
                    top_view_msg = self.bridge.cv2_to_imgmsg(self.lane_follower.image_proc.birds_image)
                    self.img_pub.publish(top_view_msg)
                    # publish Lane
                    self.publisher.publish(self.makeLane(self.lane_follower.waypoints))
                    rate.sleep()
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
    
    # make a robocar_msgs.Lane from waypoints
    def makeLane(self, waypoints):
        msg = []
        base_bottom_link = rospy.get_param('~base_frame')
        for i in range(len(waypoints)):
            new_point = Waypoint()
            # apply transformation to a pose between 'base_bottom_link' and 'world' frames            
            new_point.pose = self.transformPoint(waypoints[i], base_bottom_link, "world")
            new_point.twist.twist.linear.x = float(self.velocity)
            msg.append(new_point) 

        # form a Lane in a 'world' frame
        lane = Lane()
        lane.header.frame_id = 'world'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = msg

        return lane

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
        return new_point

    def get_point_xyz(self, point):
        p = point.pose.position        
        return p.x, p.y, p.z

    def info_cb(self, msg):
        if not self.cameraInfoSet:
            self.camera_model.fromCameraInfo(msg)
            self.cameraInfoSet = True    
            
    def img_cb(self, data):
        if (self.hasTransformMatrix):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data)
                self.lane_follower(cv_image)            
            except CvBridgeError as e:
                print(e)
            self.canTransformImg = True
 
def main(args):
    rospy.init_node('lane_follow_node', anonymous=True)
    node = LaneFollowNode()

    try:
        print("running Lane Follow Node")
    except KeyboardInterrupt:
        print("Shutting down ROS Lane Follow module")

if __name__ == '__main__':
    main(sys.argv)   