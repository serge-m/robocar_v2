#!/usr/bin/env python
import math
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TwistStamped
from ackermann_msgs.msg import AckermannDriveStamped

# Node for publishing markers for RVIZ
class MarkerPublisher(object):

    def __init__(self):
        rospy.init_node('marker_publisher', log_level=rospy.DEBUG)
        # publish markers for steering angle
        self.publish_steering()
        # publish markers for target angular velocity
        self.publish_angular_velocity()
        rospy.spin()

    def publish_steering(self):
        self.ack_pub = rospy.Publisher('/visualize/steering', Marker, queue_size=1)
        self.ack_sub = rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.ackermann_cb)        

    def ackermann_cb(self, msg):
        steering = msg.drive.steering_angle
        # publish red arrow
        self.publishMarker(steering, self.ack_pub, (1,0,0))

    def publish_angular_velocity(self):
        self.ang_pub = rospy.Publisher('/visualize/target_angular', Marker, queue_size=1)
        self.ang_sub = rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)

    def twist_cb(self, msg):
        angular_vel = msg.twist.angular.z
        # publish blue arrow
        self.publishMarker(angular_vel, self.ang_pub, (0,0,1))

    def publishMarker(self, angle, pub, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time()
        marker.ns = "/"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points.append(Point(0, 0, 0.2))
        marker.points.append(Point(math.cos(angle), math.sin(angle), 0.2))
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