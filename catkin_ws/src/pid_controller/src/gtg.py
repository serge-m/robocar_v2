#!/usr/bin/env python
# dummy go_to_goal vector generator
# only for simulator
import rospy
from std_msgs.msg import Header 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Vector3, Vector3Stamped, Point
import tf
import math
from visualization_msgs.msg import Marker

def odom_callback(data):
    global x_goal
    pos = data.pose.pose.position
    orient = data.pose.pose.orientation
    # rospy.loginfo("odometry coordinates (%s, %s) at %s",pos.x, pos.y, data.header.seq) 
    # rospy.loginfo("goal coordinates (%s, %s)",x_goal, y_goal) 
    goal_angle = (math.atan2(y_goal - pos.y, x_goal - pos.x) - orient.z + math.pi)%(2*math.pi) - math.pi
    
    vector = make_vector3_stamped(goal_angle)
    # publish vector for go to goal behavior
    pub.publish(vector)
    x_goal+=0.01

    # marker for vizualisation in RVIZ
    marker = Marker()
    marker.header.frame_id = "base"
    marker.header.stamp = rospy.Time()
    marker.ns = "/"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.points.append(Point(0, 0, 0.2))
    marker.points.append(Point(math.cos(goal_angle), math.sin(goal_angle), 0.2))
    marker.scale.x = 0.02
    marker.scale.y = 0.05
    marker.scale.z = 0.1
    marker.color.a = 1.0 # Don't forget to set the alpha!
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    vis_pub.publish(marker)

    rate.sleep()    

def make_vector3_stamped(goal_angle ):
    # create Vector3 object to goal 
    heading = Vector3(math.cos(goal_angle), math.sin(goal_angle), 1)
    # add header and create Vector3Stamped object
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base"
    vector = Vector3Stamped(header, heading)
    return vector

if __name__ == '__main__':
    x_goal, y_goal = 1.0, 0.0
    rospy.init_node('gtg', anonymous=True)
    pub = rospy.Publisher('heading/gtg', Vector3Stamped, queue_size=1)
    sub = rospy.Subscriber('robocar/map', Odometry, odom_callback)
    vis_pub = rospy.Publisher('/visualize/gtg_vectors', Marker, queue_size=1)
    
    rate = rospy.Rate(50) # 50hz
        
    rospy.spin()