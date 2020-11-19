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

class LaneFollowNode:
    def __init__(self):
        self.lane_follower = LaneFollower()
        self.camera_model = PinholeCameraModel()
        self.bridge = CvBridge()
        self.velocity = rospy.get_param('~velocity')
        # load sensor_msgs/CameraInfo Message from yaml file if it exists
        file_camera_params = rospy.get_param('~camera_params')
        if (file_camera_params):            
            camera_info_msg = camera_info_manager.loadCalibrationFile(file_camera_params, 'narrow_stereo')
            # with PinholeCameraModel we can easily get all parameters from sensor_msgs/CameraInfo Message
            if (camera_info_msg):
                c, f = self.getCameraParams(camera_info_msg)
                self.lane_follower.setCameraInfo(c, f)
                
        # get all parameters from sensor_msgs/CameraInfo Message
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_cb,  queue_size=1)
        # get camera position (in metres, degrees) from tf topic
        self.tf_listener = tf.TransformListener()
        try:
            # wait a little bit for transfrormation from base frame to camera frame
            self.tf_listener.waitForTransform("base_link", "camera", rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform("base_link", "camera", rospy.Time(0))
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
            # set camera position in LaneFollower
            self.lane_follower.setCameraPos(trans[2], pitch)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('tf error-------------------')

        # image after rectification, no need to undistort
        self.img_sub = rospy.Subscriber("/camera/color/image_rect_color", Image, self.img_cb,  queue_size=1)
        
        # publish waypoints of a lane
        self.publisher = rospy.Publisher ("/waypoints/update", Lane, queue_size=1)
        self.img_pub = rospy.Publisher ("/camera/color/top_view", Image, queue_size=1)
        
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():  
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
            # make geometry_msgs/PoseStamped Message
            p = PoseStamped()
            p.pose.position.x = float(waypoints[i][0])
            p.pose.position.y = float(waypoints[i][1])
            p.pose.position.z = float(0)
            q = tf.transformations.quaternion_from_euler(0., 0., 0)
            p.pose.orientation = Quaternion(*q)   
            p.header.frame_id = "base_link"
            p.header.stamp = rospy.Time()           
            
            new_point = Waypoint()
            # apply transformation to a pose between 'base_link' and 'world' frames
            new_point.pose = self.tf_listener.transformPose("world", p)
            new_point.twist.twist.linear.x = float(self.velocity)
            msg.append(new_point) 

        # form a Lane in a 'world' frame
        lane = Lane()
        lane.header.frame_id = 'world'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = msg

        return lane


    def info_cb(self, msg):
        c, f = self.getCameraParams(msg)
        # camera_info_manager.saveCalibration(msg, "package://robocar_simulation/config/sim_camera_calibration.yaml", "camera")
        self.lane_follower.setCameraInfo(c, f)

    def img_cb(self, data):
        try:
           cv_image = self.bridge.imgmsg_to_cv2(data)
           self.lane_follower(cv_image)            
        except CvBridgeError as e:
           print(e)
 
    def getCameraParams(self, msg):
        # load parameters from sensor_msgs/CameraInfo Message to Camera Model
        self.camera_model.fromCameraInfo(msg)
        # cx, cy - coordinates of optical center of the camera
        # fx, fy - focal lengths
        c = (self.camera_model.cx(), self.camera_model.cy())        
        f = (self.camera_model.fx(), self.camera_model.fy())
        return c, f

def main(args):
    rospy.init_node('lane_follow_node', anonymous=True)
    node = LaneFollowNode()

    try:
        print("running Lane Follow Node")
    except KeyboardInterrupt:
        print("Shutting down ROS Lane Follow module")

if __name__ == '__main__':
    main(sys.argv)   