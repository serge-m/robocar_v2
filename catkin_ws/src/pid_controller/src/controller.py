#!/usr/bin/env python
from pid import PID
import rospy


class Controller(object):
    def __init__(self):
        
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.
        mx = 0.2
        self.steer_controller = PID(kp, ki, kd, mn, mx)
        # self.throttle_controller = PID(kp, ki, kd, mn, mx)
              
        
        self.last_time = rospy.get_time()

    def control(self, current_vel, curr_ang_vel, linear_vel, angular_vel):
        
        ang_vel_error = angular_vel - curr_ang_vel
        self.last_ang_vel = curr_ang_vel

        # vel_error = linear_vel - current_vel
        # self.last_vel = current_vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        # throttle = self.throttle_controller.step(vel_error, sample_time)
        throttle = linear_vel
        steering = self.steer_controller.step(ang_vel_error, sample_time)
        
       
            
        return throttle, steering