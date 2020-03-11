#!/usr/bin/env python
# dummy go_to_goal vector generator
# only for simulator
import rospy
from std_msgs.msg import Header 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Vector3, Vector3Stamped
import tf
import math

def odom_callback(data):
    global x_goal
    pos = data.pose.pose.position
    orient = data.pose.pose.orientation
    # rospy.loginfo("odometry coordinates (%s, %s) at %s",pos.x, pos.y, data.header.seq) 
    # rospy.loginfo("goal coordinates (%s, %s)",x_goal, y_goal) 
    goal_angle = (math.atan2(y_goal - pos.y, x_goal - pos.x) - orient.z + math.pi)%(2*math.pi) - math.pi
    # create Vector3 object to goal 
    heading = Vector3(math.cos(goal_angle), math.sin(goal_angle), 1)
    # add header and create Vector3Stamped object
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base"
    vector = Vector3Stamped(header, heading)
    # publish vector for go to goal behavior
    pub.publish(vector)
    x_goal+=0.01
    rate.sleep()    

if __name__ == '__main__':
    x_goal, y_goal = 1.0, 0.0
    rospy.init_node('gtg', anonymous=True)
    pub = rospy.Publisher('heading/gtg', Vector3Stamped, queue_size=1)
    sub = rospy.Subscriber('robocar/map', Odometry, odom_callback)
    rate = rospy.Rate(50) # 50hz
        
    rospy.spin()