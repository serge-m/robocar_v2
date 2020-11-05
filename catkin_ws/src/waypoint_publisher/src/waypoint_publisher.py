#!/usr/bin/env python
import math
import numpy as np

from geometry_msgs.msg import Quaternion

from robocar_msgs.msg import Lane, Waypoint

import tf
import rospy

MAX_DECEL = 1.0


class WaypointPublisher(object):

    def __init__(self):
        rospy.init_node('waypoint_publisher', log_level=rospy.DEBUG)

        self.pub = rospy.Publisher('/waypoints/base', Lane, queue_size=1, latch=True)

        self.velocity = rospy.get_param('~velocity')
        self.new_waypoint_loader(rospy.get_param('~form'))
        rospy.spin()

    def new_waypoint_loader(self, form):        
        waypoints = self.get_waypoints(form)
        self.publish(waypoints)
        # rospy.loginfo('Waypoints published')
        # rospy.loginfo(waypoints)
        
    def quaternion_from_yaw(self):
        # TODO maybe change later 0 to smth else
        return tf.transformations.quaternion_from_euler(0., 0., 0)
    
    def get_waypoints(self, form):
        
        waypoints = []
        
        x_ = np.linspace(0, 99, 100)
        switcher={
                "straight": lambda: np.zeros_like(x_),
                "45degree": lambda: x_,
                "parabola": lambda: np.multiply(x_, x_)}
        func=switcher.get(form, lambda :np.zero(x_))

        for i in range(len(x_)):
            p = Waypoint()
            p.pose.pose.position.x = float(x_[i])
            p.pose.pose.position.y = float(func()[i])
            p.pose.pose.position.z = float(0)
            q = self.quaternion_from_yaw()
            p.pose.pose.orientation = Quaternion(*q)
            p.twist.twist.linear.x = float(self.velocity)

            waypoints.append(p)        
        return self.decelerate(waypoints)

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def decelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:
            dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        self.pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointPublisher()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')