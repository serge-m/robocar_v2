#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Header, Float32 
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import PointStamped, Point, Vector3, Vector3Stamped, Twist
from pid_controller import PIDController
import tf
import math
from visualization_msgs.msg import Marker

def at_obstacle(dist):
    return dist < 1

def callback(ao_vector, lf_vector, *argv):
    distance_array = []
    for sensor in argv:
        distance_array.append(sensor.range)
    obst_dist = min(distance_array)
    heading_vector = lf_vector.vector

    # combine two vectors into one with coeff
    # alpha coeff if sensors are in a circle
    alpha = 0.6
    # alpha coeff if sensors are in one direction
    # alpha = 0.3
    heading_vector.x = alpha*lf_vector.vector.x + (1-alpha)*ao_vector.vector.x
    heading_vector.y = alpha*lf_vector.vector.y + (1-alpha)*ao_vector.vector.y
    
    # marker for vizualisation in RVIZ
    marker = Marker()
    marker.header.frame_id = "base"
    marker.header.stamp = rospy.Time()
    marker.ns = "/"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.points.append(Point(0, 0, 0.2))
    marker.points.append(Point(heading_vector.x, heading_vector.y, 0.2))
    marker.scale.x = 0.02
    marker.scale.y = 0.05
    marker.scale.z = 0.1
    marker.color.a = 1.0 # Don't forget to set the alpha!
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    vis_pub.publish(marker)
    
    if at_obstacle(obst_dist):
        heading_vector = ao_vector.vector
        rospy.loginfo("use avoid obstacles vector %s",heading_vector)
    
    vel_msg = Twist()
    # Linear velocity in the x-axis.
    # TODO do someting with velocity
    vel_msg.linear.x = v
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    # Angular velocity in the z-axis.
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    global last_time
    current_time = rospy.Time.now()
    dt = current_time.to_time() - last_time.to_time()
    vel_msg.angular.z = pid.execute(heading_vector, dt)
    last_time = rospy.Time.now()
    # Publishing our vel_msg
    pub.publish(vel_msg)

    rate.sleep()               
       
    
def control():
    # 2 subscribers for 2 vectors
    ao_vector = message_filters.Subscriber("heading/avoid_obstacles", Vector3Stamped)
    # ADD lane_follower vector
    # lf_vector = message_filters.Subscriber("lane_follow/desired_shift", Vector3Stamped)
    # only for simulator use this dummy vector
    lf_vector = message_filters.Subscriber("heading/gtg", Vector3Stamped)
    # 4 subscribers for ultrasonic scans
    left = message_filters.Subscriber("scan/left_ultra_sonic", Range)
    left_center = message_filters.Subscriber("scan/left_center_ultra_sonic", Range)
    right_center = message_filters.Subscriber("scan/right_center_ultra_sonic", Range)
    right = message_filters.Subscriber("scan/right_ultra_sonic", Range)
    # combine all in one callback
    ts = message_filters.ApproximateTimeSynchronizer([ao_vector, lf_vector, left, left_center, right_center, right], 1, 0.1)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':

    rospy.init_node('pid_supervisor', anonymous=True)
    # TODO maybe get this from ParamsServer
    kp, ki, kd = 4.0, 0.01, 0.0
    v = 0.5
    last_time = rospy.Time.now()
    pid = PIDController(kp, ki, kd)
    t = tf.TransformListener()
    pub = rospy.Publisher('/robocar/cmd_vel', Twist, queue_size=1)
    vis_pub = rospy.Publisher('/visualize/blend_vectors', Marker, queue_size=1)
    
    rate = rospy.Rate(50) # 10hz
    
    control()