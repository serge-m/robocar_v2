#!/usr/bin/env python
import numpy as np

from img_helpers import BirdsEye, Treshold, FitPolynomial

# class uses camera parameters and image
# to calculate new waypoints for following the lane
class LaneFollower:
    def __init__(self, Wx=4.8, Wy = 3.6, s=100):
        # F = (fx, fy) - focal length (in pixels)
        # C = (cx, cy) - coordinates of optical center of the camera (in pixels)        
        # Defining desired field-of-view (in metres)
        # W = (Wx, Wy) are the width and height of the top-view image
        #     x-axis is the direction pointing right in the top-view image
        #     y-axis is the direction pointing down in the top-view image
        # O = (Ox, Oy) is the position of the projection of the optical center on the 
        #     ground plane from (0, 0) of the top-view image
        # S - Scaling factor (in pixels/m)
        #     (Use 80.0 for realtime low-res output but 160.0 for datasets)
        # H, theta - Camera position (in metres, radians)
        self.F = None        
        self.C = None
        self.W = (Wx, Wy)
        self.O = (Wx/2, Wy)
        self.S = s
        self.H = None
        self.theta = 0.0

        # treshold image
        self.TresholdImage = Treshold()
        # warp it to the birds eye projection
        self.birdsEyeImage = None        
        # use polynomials for left, center and right lines to find new waypoints
        self.FitPolynomial = FitPolynomial()
        # waypoints to return
        self.waypoints = None

    def setCameraInfo(self, c=None, f=None):
        if (self.C != c) or (self.F != f):
            self.C = c
            self.F = f
            if self.H:
                self.birdsEyeImage = BirdsEye(self.F, self.C, self.H, self.theta, self.O, self.W, self.S)

    def setCameraPos(self, z, pitch):
        self.H = z
        self.theta = np.pi /2 - pitch 

    def __call__(self, image):
        waypoints = [[1, 0]]
        if (self.birdsEyeImage is None) and self.F and self.C and self.H:
            self.birdsEyeImage = BirdsEye(self.F, self.C, self.H, self.theta, self.O, self.W, self.S)            
            
        # get waypoints from the image
        self.TresholdImage.treshold_binary(image)
        if (np.count_nonzero(self.TresholdImage.image) > 0):
            self.birdsEyeImage.warpPerspective(self.TresholdImage.image) 
            if (np.count_nonzero(self.birdsEyeImage.birds_image) > 0):
                self.FitPolynomial.fit_polynomial(self.birdsEyeImage.birds_image)
                if (self.FitPolynomial.left_fit is not None) and (self.FitPolynomial.right_fit is not None):
                    self.waypoints = self.FitPolynomial.get_waypoints()     
        return self.waypoints

