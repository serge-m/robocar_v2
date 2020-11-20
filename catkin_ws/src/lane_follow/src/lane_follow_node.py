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

        # load sensor_msgs/CameraInfo Message from yaml file if it exists
        file_camera_params = rospy.get_param('~camera_params')
        self.cam_info_mng = camera_info_manager.CameraInfoManager(cname='narrow_stereo', url=file_camera_params)
        self.cam_info_mng.loadCameraInfo()
        # if there is no yaml file msg is equal to null, no error msg
        camera_info_msg = self.cam_info_mng.getCameraInfo()              
        if (camera_info_msg):  
            self.info_cb(camera_info_msg)
                
        # get camera position (in metres, degrees) from tf topic
        self.tf_listener = tf.TransformListener()
        # try:
        #     # wait a little bit for transfrormation from base frame to camera frame
        #     self.tf_listener.waitForTransform("base_link", "camera", rospy.Time(0), rospy.Duration(4.0))
        #     (trans, rot) = self.tf_listener.lookupTransform("base_link", "camera", rospy.Time(0))
        #     (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
        #     # set camera position in LaneFollower
        #     self.lane_follower.setCameraPos(trans[2], pitch)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print('tf error-------------------')

        # get all parameters from sensor_msgs/CameraInfo Message
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_cb,  queue_size=1)
        
        # image after rectification, no need to undistort
        self.img_sub = rospy.Subscriber("/camera/color/image_rect_color", Image, self.img_cb,  queue_size=1)
        
        # publish waypoints of a lane
        self.publisher = rospy.Publisher ("/waypoints/update", Lane, queue_size=1)
        self.img_pub = rospy.Publisher ("/camera/color/top_view", Image, queue_size=1)
        
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown(): 
            # TODO test for null values 
            # if it is true - camera model is loaded
            if self.cam_info_mng.getCameraInfo():
                try:
                    # find src and dst points for 
                    # LaneFollower.image_proc.get_transform_matrix(src, dst)
                    # it is done here because points 
                    # should be transformed between frames
                    h = self.cam_info_mng.getCameraInfo().height
                    w = self.cam_info_mng.getCameraInfo().width
                    # left bottom corner        
                    lbc_ray = self.camera_model.projectPixelTo3dRay((0, h))
                    # right bottom corner        
                    rbc_ray = self.camera_model.projectPixelTo3dRay((w, h))
                    # camera center and bottom points in the base_link frame
                    zero = self.get_point_xyz(self.transformPoint((0,0,0), "camera_link_optical", "base_link"))
                    lbc_point = self.get_point_xyz(self.transformPoint(lbc_ray, "camera_link_optical", "base_link"))
                    rbc_point = self.get_point_xyz(self.transformPoint(rbc_ray, "camera_link_optical", "base_link"))
                    # TODO get -0.092 from tf or somewhere else
                    # ground plane in the base_link frame
                    xoy = (0, 0, -0.092)
                    scale = 2*h/w
                    point3, point4 = getUpperPoints(zero, lbc_point, rbc_point, xoy, scale)
                    # transform points 3 and 4 to camera_optical_link frame
                    luc_point = self.get_point_xyz(self.transformPoint(point3, "base_link", "camera_link_optical"))
                    ruc_point = self.get_point_xyz(self.transformPoint(point4, "base_link", "camera_link_optical"))
                    # pixel coordinates of these points:
                    # left upper corner 
                    luc = self.camera_model.project3dToPixel(luc_point)
                    # right upper corner        
                    ruc = self.camera_model.project3dToPixel(ruc_point)
                    # form src and dst points
                    src = [[luc],[ruc],[w, h],[0, h]]
                    dst = [[0, 0],[w, 0],[w, h],[0, h]]
                    # get matrix for perspective transformation
                    self.lane_follower.image_proc.get_transform_matrix(src, dst)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

                
            if (self.lane_follower.waypoints is not None): 
                # publish warped tresholded image
                top_view_msg = self.bridge.cv2_to_imgmsg(self.lane_follower.image_proc.birds_image)
                self.img_pub.publish(top_view_msg)
                # publish Lane
                # self.tf_listener.waitForTransform("world", "base_link", rospy.Time.now(), rospy.Duration(4.0))
                self.publisher.publish(self.makeLane(self.lane_follower.waypoints))
                rate.sleep()
    
    # make a robocar_msgs.Lane from waypoints
    def makeLane(self, waypoints):
        msg = []
        
        for i in range(len(waypoints)):
            new_point = Waypoint()
            # apply transformation to a pose between 'base_link' and 'world' frames
            new_point.pose = self.transformPoint(waypoints[i], "base_link", "world")
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
        self.camera_model.fromCameraInfo(msg)
        # TODO test if there is no calibration file
        if not self.cam_info_mng.getCameraInfo():
            self.cam_info_mng.saveCalibration(msg, file_camera_params, 'narrow_stereo')
            self.cam_info_mng.loadCameraInfo()
            
    def img_cb(self, data):
        try:
           cv_image = self.bridge.imgmsg_to_cv2(data)
           self.lane_follower(cv_image)            
        except CvBridgeError as e:
           print(e)
 
def main(args):
    rospy.init_node('lane_follow_node', anonymous=True)
    node = LaneFollowNode()

    try:
        print("running Lane Follow Node")
    except KeyboardInterrupt:
        print("Shutting down ROS Lane Follow module")

if __name__ == '__main__':
    main(sys.argv)   