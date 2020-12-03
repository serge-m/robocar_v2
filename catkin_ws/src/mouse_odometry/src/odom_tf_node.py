#!/usr/bin/env python  

# Node that publishes transformations 
# between 'world' and 'base_link' frames in /tf
import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion

def tf_pub_cb(msg):
    br = tf.TransformBroadcaster()
    odom_trans = TransformStamped()
    odom_trans.header.stamp = rospy.Time.now()
    odom_trans.header.frame_id = "world"
    odom_trans.child_frame_id = "base_bottom_link"

    odom_trans.transform.translation.x = msg.pose.pose.position.x
    odom_trans.transform.translation.y = msg.pose.pose.position.y
    # TODO make this dependant from urdf file with car description
    odom_trans.transform.translation.z = 0
    
    odom_trans.transform.rotation = msg.pose.pose.orientation

    # send the transform
    br.sendTransformMessage(odom_trans)
    

if __name__ == '__main__':
    rospy.init_node('odom_tf_node', anonymous=True)
    rospy.Subscriber("robocar/odometry", Odometry, tf_pub_cb,  queue_size=1)
    rospy.spin()