import math

class YawController(object):
    def __init__(self, axle_dist, min_speed, max_steer_angle):
        self.axle_dist = axle_dist
        self.min_speed = min_speed
        
        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle


    def get_angle(self, angular_vel, linear_vel):
        # use formula of dependency between steering angle, linear  
        # and angular velocities for vehicles with ackermann steering
        # w = (v*tan(phi))/l
        # w - angular velocity
        # v - linear velocity
        # l - distance between front and rear axles
        # phi - steering angle
        angle = math.atan2(angular_vel*self.axle_dist, linear_vel)
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        return self.get_angle(angular_velocity, max(current_velocity, self.min_speed)) if abs(angular_velocity) > 0. else 0.0

        