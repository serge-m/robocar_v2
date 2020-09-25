#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

import math

from controller import Controller

# TODO load from parameters
MIN_STEER_ANGLE = -0.52359878
MAX_STEER_ANGLE = 0.52359878

class SpeedAdjuster(object):
    """
     ESC of the RC-car treats switching the direction of movement as 
     a power break. We have to do a power break, then send 0, then send new speed
    """

    def __init__(self):
        self.speed_to_publish = []
        self.prev_speed = 0

    def __call__(self, speed):
        # rospy.loginfo("speed {}, prev {}, pub {}".format(speed, self.prev_speed, self.speed_to_publish))
        if self.speed_to_publish:
            return self.speed_to_publish.pop()
        if self.prev_speed * speed >= 0:
            self.prev_speed = speed
            return speed
        
        self.speed_to_publish = (
            [0,] * 20  # stop
            + [0.1 * speed / abs(speed),]*20 # reverse
        )
        self.prev_speed = 0
        return 0

class ControlSupervisor(object):
    def __init__(self):
        rospy.init_node('control_supervisor_node')
 
        self.ackermann_sim_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=1)
        
        self.controller = Controller()
        self.speed_adjuster = SpeedAdjuster()
        # target linear and angular velocities
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        # current pose and velocities
        rospy.Subscriber('/robocar/odometry', Odometry, self.odom_cb)
        
        self.current_vel = None
        self.curr_ang_vel = None
        self.linear_vel = None
        self.angular_vel = None
        self.throttle = self.steering = 0
        # TODO load from urdf or params
        self.axle_dist = 0.258
        
        self.loop()

    def loop(self):
        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():           
            if not None in (self.current_vel, self.linear_vel, self.angular_vel):
                # self.throttle, self.steering = self.controller.control(self.current_vel, self.curr_ang_vel, self.linear_vel, self.angular_vel)                        
                # rospy.loginfo("curr_ang_vel is %s, target angular_vel is %s", self.curr_ang_vel, self.angular_vel)
                # rospy.loginfo("steering is %s", self.steering)
                # Still without controller, just use twist_cmd from waypoint_follower
                self.throttle = self.linear_vel
                self.steering = math.atan2(self.angular_vel*self.axle_dist, self.linear_vel)
                if (self.steering < MIN_STEER_ANGLE):
                    self.steering = MIN_STEER_ANGLE
                if (self.steering > MAX_STEER_ANGLE):
                    self.steering = MAX_STEER_ANGLE
                # rospy.loginfo("steering is %s", self.steering)
                self.publish(self.throttle, self.steering)
            rate.sleep()
    
    def twist_cb(self, msg):
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z
        
    def odom_cb(self, msg):
        self.current_vel = msg.twist.twist.linear.x
        self.curr_ang_vel = msg.twist.twist.angular.z
            
    def publish(self, throttle, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        msg.drive.speed = self.speed_adjuster(throttle)
        msg.drive.acceleration = 1
        msg.drive.jerk = 1
        msg.drive.steering_angle = steer
        msg.drive.steering_angle_velocity = 1
        
        self.ackermann_sim_pub.publish(msg)


if __name__ == '__main__':
    ControlSupervisor()