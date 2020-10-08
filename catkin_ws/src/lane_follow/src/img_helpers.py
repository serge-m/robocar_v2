#!/usr/bin/env python
import numpy as np
import cv2
import math

# Class for perspective transformation of an image
class BirdsEye:
    # F = (fx, fy) - focal length (in pixels)
    # C = (cx, cy) - coordinates of optical center of the camera (in pixels)
    # H, theta - Camera position (in metres, radians)
    # Defining desired field-of-view (in metres)
    # O = (Ox, Oy) is the position of the projection of the optical center on the 
    # ground plane from (0, 0) of the top-view image
    # W = (Wx, Wy) are the width and height of the top-view image
    # x-axis is the direction pointing right in the top-view image
    # y-axis is the direction pointing down in the top-view image
    # s - Scaling factor (in pixels/m)
    # (Use 80.0 for realtime low-res output but 160.0 for datasets)
    def __init__(self, F, C, H, theta, O, W, s):
      
        self.transform = np.zeros((3,3))
        # Calculating transformation matrix analytically
        self.transform[0][0] = F[0]
        self.transform[0][1] = -C[0]*math.cos(theta)
        self.transform[0][2] = s*(C[0]*(H*math.sin(theta)+O[1]*math.cos(theta)) - O[0]*F[0])

        self.transform[1][0] = 0
        self.transform[1][1] = F[1]*math.sin(theta) - C[1]*math.cos(theta)
        self.transform[1][2] = s*(C[1]*(H*math.sin(theta)+O[1]*math.cos(theta)) + F[1]*(H*math.cos(theta)-O[1]*math.sin(theta)))

        self.transform[2][0] = 0
        self.transform[2][1] = -math.cos(theta)
        self.transform[2][2] = s*(H*math.sin(theta) + O[1]*math.cos(theta))

        # Normalizing transformation matrix
        self.transform = self.transform/self.transform[2][2]
        print('Transformation Matrix----------------------------')
        print(self.transform)
        self.birds_size = (int(s*W[0]), int(s*W[1]))
        self.birds_image = np.zeros(self.birds_size)

    def warpPerspective(self, img):
        self.birds_image = cv2.warpPerspective(img, self.transform, self.birds_size, flags=
				cv2.INTER_LINEAR+cv2.WARP_FILL_OUTLIERS+cv2.WARP_INVERSE_MAP)