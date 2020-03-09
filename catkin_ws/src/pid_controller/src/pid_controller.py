#!/usr/bin/env python
import time
import math

class PIDController:
    
    def __init__(self, kp, ki, kd):
        '''Initialize internal variables'''        
        self.heading_angle = 0
        self.set_parameters(kp, ki, kd)
        self.restart()
    
    def restart(self):
        """Set the integral and differential errors to zero"""
        self.E = 0
        self.error_1 = 0
        self._last_time = time.time() # Used for automatic calculation of dt.

    def set_parameters(self, kp, ki, kd):
        """Set PID values        
        :param params.gains.kp: Proportional gain
        :type params.gains.kp: float
        :param params.gains.ki: Integral gain
        :type params.gains.ki: float
        :param params.gains.kd: Differential gain
        :type params.gains.kd: float
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        
    def get_heading_angle(self, heading_vector):
        # Return the heading as an angle in the robot's frame of reference.
        # heading_vector is Vector3Stamped object

        return math.atan2(heading_vector.y,heading_vector.x)        
    
    def execute(self, heading_vector, dt):
        
        # This is the direction we want to go
        self.heading_angle = self.get_heading_angle(heading_vector)

        # Calculate simple proportional error
        # The direction is in the robot's frame of reference, so the error is the direction.
        error = self.heading_angle

        # Calculate integral error
        self.E += error*dt
        self.E = (self.E + math.pi)%(2*math.pi) - math.pi

        # Calculate differential error
        dE = (error - self.error_1)/dt
        self.error_1 = error #updates the error_1 var

        # Calculate desired w
        w = self.kp*error + self.ki*self.E + self.kd*dE
     
        return w