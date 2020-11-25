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

    def setCameraInfo(self, c=None, f=None):
        return
        # if (self.C != c) or (self.F != f):
        #     self.C = c
        #     self.F = f
        #     if self.H:
        #         self.birdsEyeImage = BirdsEye(self.F, self.C, self.H, self.theta, self.O, self.W, self.S)

    def setCameraPos(self, z, pitch):
        return
        # self.H = z
        # self.theta = pitch 

    def __call__(self, image):
        waypoints = [[1, 0, 0]]
        # if (self.birdsEyeImage is None) and self.F and self.C and self.H:
        #     self.birdsEyeImage = BirdsEye(self.F, self.C, self.H, self.theta, self.O, self.W, self.S)            
            
        # # get waypoints from the image
        # self.TresholdImage.treshold_binary(image)
        # if (np.count_nonzero(self.TresholdImage.image) > 0):
        #     self.birdsEyeImage.warpPerspective(self.TresholdImage.image) 
        #     if (np.count_nonzero(self.birdsEyeImage.birds_image) > 0):
        #         self.FitPolynomial.fit_polynomial(self.birdsEyeImage.birds_image)
        #         if (self.FitPolynomial.left_fit is not None) and (self.FitPolynomial.right_fit is not None):
        #             self.waypoints = self.FitPolynomial.get_waypoints() 


        if (self.image_proc.transformMatrix is not None):
            treshold_image = self.image_proc.treshold_binary(image)

            # h, w = image.shape[0], image.shape[1]         
            # src = np.float32([[387, 525],[894, 525],[w, h],[0, h]])
            # dst = np.float32([[0, 0],[w, 0],[w, h],[0, h]])
            # self.image_proc.get_transform_matrix(src, dst)

            birds_image = self.image_proc.warp_perspective(treshold_image)
            
            left_fit, right_fit, poly_img = self.image_proc.fit_polynomial(birds_image)
            
            self.waypoints = self.image_proc.get_waypoints()    

        return self.waypoints

