#!/usr/bin/env python
"""
Helper node that waits for the user input from rviz tool  "2D Nav Goal" published
on /move_base_simple/goal, aggregates them in a sequence and publishes it to /waypoints topic
as PoseArray.
"""
from collections import OrderedDict
from geometry_msgs.msg import PoseStamped, PoseArray
import rospy


class WaypointMemo(object):

    def __init__(self):
        rospy.init_node('waypoint_memo', anonymous=True, log_level=rospy.INFO)
        self.pub = rospy.Publisher('/waypoints', PoseArray, queue_size=1)
        self.sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.add_waypoint, queue_size=1)
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        self.timeout = rospy.Duration.from_sec(float(rospy.get_param('~expiration_time', 3.0)))
        self.cur_waypoint_id = 0
        self.waypoints = OrderedDict()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = self._create_pose_array()
            self.pub.publish(msg)
            rospy.logdebug('Published {}'.format(msg.poses))
            rate.sleep()
            rospy.logdebug('after sleep {}'.format(msg.poses))

    def _create_pose_array(self):
        msg = PoseArray()
        now = rospy.Time.now()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.poses = [stamped_pose.pose for stamped_pose in self.waypoints.values()
                     if now - stamped_pose.header.stamp < self.timeout]
        return msg

    def add_waypoint(self, msg):
        """
        :type msg: PoseStamped
        """
        now = rospy.Time.now()
        self.waypoints = OrderedDict(
            (k, v) for k, v in self.waypoints.items()
            if v.header.stamp if now - v.header.stamp < self.timeout
        )
        self.waypoints[self.cur_waypoint_id] = msg

        self.cur_waypoint_id += 1


if __name__ == '__main__':
    try:
        wm = WaypointMemo()
        wm.run()
    except rospy.ROSInterruptException:
        rospy.logerr('Processing interrupted')
