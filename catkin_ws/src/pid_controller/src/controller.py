#!/usr/bin/env python
from pid import PID
from yaw_controller import YawController
import time

class Controller(object):
    def __init__(self, axle_dist, max_velocity, max_steer_angle):
        
        self.axle_dist = axle_dist
        self.last_time = time.time()
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0
        mx = max_velocity
        self.throttle_controller = PID(kp, ki, kd, mn, mx)        
        self.yaw_controller = YawController(self.axle_dist, 0.1, max_steer_angle)

    def control(self, current_vel, curr_ang_vel, linear_vel, angular_vel):
           
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        current_time = time.time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        if (sample_time == 0):
            throttle = 0
        else:
            throttle = self.throttle_controller.step(vel_error, sample_time)
       
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        return throttle, steering