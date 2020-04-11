#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Header, Float32 
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import PointStamped, Point, Vector3, Vector3Stamped, Twist
from pid_controller import PIDController, get_heading_angle
import tf
import math
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDrive
from pwm_radio_arduino.msg import steering_tfm
import threading
import numpy as np
import time


def callback(ao_vector, gtg_vector, at_obstacle):

    heading_vector = gtg_vector.vector

    # combine two vectors into one with coeff
    # alpha coeff if sensors are in a circle
    alpha = 0.6
    # alpha coeff if sensors are in one direction
    # alpha = 0.3
    heading_vector.x = alpha*gtg_vector.vector.x + (1-alpha)*ao_vector.vector.x
    heading_vector.y = alpha*gtg_vector.vector.y + (1-alpha)*ao_vector.vector.y
    
    
    marker = make_marker_for_rviz(heading_vector)
    vis_pub.publish(marker)
    
    if at_obstacle.vector.x != 0.:  
        heading_vector = ao_vector.vector
        rospy.loginfo("use avoid obstacles vector %s",heading_vector)
    

    global last_time
    current_time = rospy.Time.now()
    dt = current_time.to_time() - last_time.to_time()
    
    if dt > 0:
        vel_msg = make_vel_msg(heading_vector, dt)
        last_time = rospy.Time.now()
        pub.publish(vel_msg)
        rospy.logdebug("pub vel_msg {}".format(vel_msg))
        
        angle = get_heading_angle(heading_vector) / (3.14 / 2) + 0.3 * np.sin(rospy.get_time() * 2 % (2 * 3.14) )  
        msg = AckermannDrive(speed=v, steering_angle=angle)
        pub_ackermann.publish(msg)

    rate.sleep()               

def make_vel_msg(heading_vector, dt):
    vel_msg = Twist()
    # Linear velocity in the x-axis.
    # TODO do someting with velocity
    vel_msg.linear.x = v
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    # Angular velocity in the z-axis.
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = pid.execute(heading_vector, dt)
    return vel_msg

def make_marker_for_rviz(heading_vector):
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
    return marker
       
    
def control():
    # 2 subscribers for 2 vectors
    ao_vector = message_filters.Subscriber("heading/avoid_obstacles", Vector3Stamped)
    # ADD lane_follower vector
    # lf_vector = message_filters.Subscriber("lane_follow/desired_shift", Vector3Stamped)
    # only for simulator use this dummy vector
    gtg_vector = message_filters.Subscriber("heading/gtg", Vector3Stamped)
    # 4 subscribers for ultrasonic scans
    at_obstacle = message_filters.Subscriber("heading/at_obstacle", Vector3Stamped)
    # combine all in one callback
    ts = message_filters.ApproximateTimeSynchronizer([ao_vector, gtg_vector, at_obstacle], 1, 0.1)
    ts.registerCallback(callback)
    
    rospy.spin()




def start_thread_pub_tfm_params():
    thread = threading.Thread(target=pub_tfm_params,args=())
    thread.setDaemon(True)
    thread.start()

def pub_tfm_params():
    pub_tfm = rospy.Publisher("pwm_radio_arduino/steering_tfm", steering_tfm, queue_size=1, latch=False)
    while not rospy.is_shutdown():
        tfm = steering_tfm(
            -1, 0, 1,           # input range for angle
            90-30, 90, 90+30,   # output range for angle
            -1, 0, 1,           # input range for speed
            90-1, 90, 90+1      # output range for speed
        )
        pub_tfm.publish(tfm)  
        rospy.logdebug("pub_tfm")
        time.sleep(1) 


if __name__ == '__main__':

    rospy.init_node('pid_supervisor', anonymous=True)

    start_thread_pub_tfm_params()

    # TODO maybe get this from ParamsServer
    kp, ki, kd = 4.0, 0.01, 0.0
    v = 0.5
    last_time = rospy.Time.now()
    pid = PIDController(kp, ki, kd)
    t = tf.TransformListener()
    pub = rospy.Publisher('/robocar/cmd_vel', Twist, queue_size=1)
    vis_pub = rospy.Publisher('/visualize/blend_vectors', Marker, queue_size=1)
    rate = rospy.Rate(50) # 10hz
    pub_ackermann = rospy.Publisher("pwm_radio_arduino/driver_ackermann", AckermannDrive, queue_size=1, latch=False)
    control()