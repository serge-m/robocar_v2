#!/usr/bin/env python
import numpy as np

from img_helpers import ImageProcessor

# class uses camera parameters and image
# to calculate new waypoints for following the lane
class LaneFollower:
    def __init__(self):
        self.image_proc = ImageProcessor()
        # waypoints to return
        self.waypoints = None

    def __call__(self, image):
        waypoints = [[1, 0, 0]]
        
        if (self.image_proc.transformMatrix is not None):
            # treshold image
            treshold_image = self.image_proc.treshold_binary(image)
            # warp it to the top-view projection
            birds_image = self.image_proc.warp_perspective(treshold_image)
            # find polynolmials for left and right lane lines
            left_fit, right_fit, poly_img = self.image_proc.fit_polynomial(birds_image)
            # find waypoints from middle line
            self.waypoints = self.image_proc.get_waypoints()    

        return self.waypoints

