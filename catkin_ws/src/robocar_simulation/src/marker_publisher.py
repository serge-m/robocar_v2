#!/usr/bin/env python
import math
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TwistStamped
from ackermann_msgs.msg import AckermannDriveStamped
from robocar_msgs.msg import Lane, Waypoint

# Node for publishing markers for RVIZ
class MarkerPublisher(object):

    def __init__(self):
        rospy.init_node('marker_publisher', log_level=rospy.DEBUG)
        # publish markers for steering angle
        self.publish_steering()
        # publish markers for target angular velocity
        self.publish_angular_velocity()
        # publish lane waypoints
        self.publish_waypoints()
        rospy.spin()

    def publish_steering(self):
        self.ack_pub = rospy.Publisher('/visualize/steering', Marker, queue_size=1)
        self.ack_sub = rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.ackermann_cb)        

    def ackermann_cb(self, msg):
        steering = msg.drive.steering_angle
        # publish red arrow
        self.publishMarker(self.ack_pub, marker_type = "arrow", angle=steering, color=(1,0,0))

    def publish_angular_velocity(self):
        self.ang_pub = rospy.Publisher('/visualize/target_angular', Marker, queue_size=1)
        self.ang_sub = rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)

    def twist_cb(self, msg):
        angular_vel = msg.twist.angular.z
        # publish blue arrow
        self.publishMarker(self.ang_pub, marker_type="arrow", angle=angular_vel, color=(0,0,1))

    def publish_waypoints(self):
        self.waypoint_pub = rospy.Publisher('/visualize/waypoints', Marker, queue_size=1)
        self.waypoint_sub = rospy.Subscriber('/waypoints/update', Lane, self.waypoint_cb)

    def waypoint_cb(self, msg):
        waypoints = msg.waypoints
        points = [x.pose.pose.position for x in waypoints]
        self.publishMarker(self.waypoint_pub, marker_type="line", points=points, color=(0,1,0), frame="world")

    def publishMarker(self, pub, marker_type, color, angle=0, points=None, frame="base_link"):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time()
        marker.ns = "/"
        marker.id = 0        
        marker.action = Marker.ADD        
        if (marker_type == "arrow"):
            marker.type = Marker.ARROW
            marker.points.append(Point(0, 0, 0))
            marker.points.append(Point(math.cos(angle), math.sin(angle), 0))
        else:
            marker.type = Marker.LINE_STRIP
            marker.points.extend(points)
        marker.scale.x = 0.02
        marker.scale.y = 0.05
        marker.scale.z = 0.1
        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        pub.publish(marker)


if __name__ == '__main__':
    try:
        MarkerPublisher()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')