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

class GoalBase(object):
    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y
        
class GoalForward(GoalBase):
    def __init__(self):
        self._x = 1.0
        self._y = 0.0

    def update(self, robot_position):
        self._x = robot_position.x + 1.


class GoalLoop(GoalBase):
    def __init__(self, x_limit):
        self._x = x_limit
        self._y = 0.0
        self.direction = 0
        self.further_limit = x_limit 
        self.tolerance = 0.1
        self.near_limit = 0

    def update(self, robot_position):
        if self.direction == 0:  # going forward
            if robot_position.x + self.tolerance >= self._x :
                self.direction = 1
                self._x = self.near_limit 
                self._y = 0.
                rospy.loginfo("GoalLoop - position {pos.x} {pos.y}, new direction {direction}, new goal {goal.x} {goal.y}".format(
                    pos=robot_position, direction=self.direction, goal=self))
            return

        if self.direction == 1:  # going backward
            if robot_position.x <= self.near_limit:
                self.direction = 0
                self._x = self.further_limit
                self._y = 0.
                rospy.loginfo("GoalLoop - position {pos.x} {pos.y}, new direction {direction}, new goal {goal.x} {goal.y}".format(
                    pos=robot_position, direction=self.direction, goal=self))
            return

        raise RuntimeError("unsupported direction {}".format(self.direction))


def odom_callback(data):
    pos = data.pose.pose.position
    orient = data.pose.pose.orientation
    
    dist = ((goal.x - pos.x)**2 + (goal.y-pos.y)**2)**0.5
    goal_angle = (math.atan2(goal.y - pos.y, goal.x - pos.x) - orient.z + math.pi)%(2*math.pi) - math.pi
    
    rospy.loginfo_throttle(10, "Goal ({}, {}), odometry ({}, {}) orient {} at {} -> goal_angle {:.3f}".format(
        goal.x, goal.y, pos.x, pos.y, orient.z, data.header.seq, goal_angle))


    vector = make_vector3_stamped(goal_angle)
    # publish vector for go to goal behavior
    pub.publish(vector)
    goal.update(pos)

    marker = make_marker_for_rviz(goal_angle)
    vis_pub.publish(marker)

    rate.sleep()    

def make_marker_for_rviz(goal_angle):
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
    return marker

def make_vector3_stamped(goal_angle ):
    # create Vector3 object to goal 
    heading = Vector3(math.cos(goal_angle), math.sin(goal_angle), 0)
    # add header and create Vector3Stamped object
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base"
    vector = Vector3Stamped(header, heading)
    return vector

if __name__ == '__main__':
    rospy.init_node('gtg', anonymous=True)

    mode = rospy.get_param('~mode','forward')
    if mode == 'forward':
        goal = GoalForward()
    elif mode == 'loop':
        goal = GoalLoop(1.0)
    else:
        raise NotImplementedError("mode '{}' is not supported".format(mode))

    rospy.loginfo("Running gtg node with mode '{}'".format(mode))
    
    pub = rospy.Publisher('heading/gtg', Vector3Stamped, queue_size=1)
    sub = rospy.Subscriber('robocar/odometry', Odometry, odom_callback)
    vis_pub = rospy.Publisher('/visualize/gtg_vectors', Marker, queue_size=1)
    
    rate = rospy.Rate(50) # 50hz
        
    rospy.spin()