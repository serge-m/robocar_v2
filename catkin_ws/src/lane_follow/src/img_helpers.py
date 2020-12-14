#!/usr/bin/env python
import numpy as np
import cv2
import math
from sympy import Point3D, Line3D, Plane 

# get upper src points in the base_link frame
# transformed points are used in 
# ImageProcessor.get_transform_matrix(src, dst)
# for (un)wrapping image into top-view perspective.
# All points are in the base_link frame
# zero - camera center
# lbc - ray which goes from center and left bottom corner of the image
# rbc - ray which goes from center and right bottom corner of the image
# xoy - any point which lay on the ground
# y_scale - scale factor, meters per image height
def getUpperPoints(zero, lbc, rbc, y_scale=2, xoy=(0, 0, 0)):
    # make two lines from camera center
    lbc_line = Line3D(Point3D(zero), Point3D(lbc))
    rbc_line = Line3D(Point3D(zero), Point3D(rbc))
    # ground plane with lanes
    xy_plane = Plane(Point3D(xoy), normal_vector=(0, 0, 1))
    # bottom points in the base_bottom_link frame
    # are intersection points of lines and ground plane
    point1 = xy_plane.intersection(lbc_line)[0]
    point2 = xy_plane.intersection(rbc_line)[0]
    # translate factor in meters
    # depends on h*w of the image
    # and scale factor
    x_scale = float(point1.distance(point2))
    # upper points in the base_link frame
    point3 = point1.translate(y_scale)
    point4 = point2.translate(y_scale)
    return point3, point4, x_scale

# class for processing images:
# - top-view perspective transformation
# - gradient and color tresholds
# - finding lane lines and fitting polynomials
# - getting waypoints from middle line in the base_link frame in meters
class ImageProcessor:
    def __init__(self, nwindows=30, margin=100, minpix=50):
        # transformation matrix for 
        # (un)wraping images to top-view projection
        self.transformMatrix = None
        # top-view image
        self.birds_image = None
        # tresholded image
        self.treshold_image = None
        # polynomials for left, right and center lines
        self.left_fit = None
        self.right_fit = None
        self.center_array = None
        # shape of an image
        self.shape = None
        # image with drawn polygon between left and right lines
        self.poly_img = None
        # HYPERPARAMETERS
        # Choose the number of sliding windows
        self.nwindows = nwindows
        # Set the width of the windows +/- margin
        self.margin = margin
        # Set minimum number of pixels found to recenter window
        self.minpix = minpix
        # (x_scale, y_scale) - scale factor in m per pixel
        self.scale = None

    def setScale(self, scale):
        self.scale = scale

    def get_transform_matrix(self, src, dst):
        self.transformMatrix = cv2.getPerspectiveTransform(np.float32(src), np.float32(dst)) 

    # get_transform_matrix(src, dst) should be called before
    def warp_perspective(self, img):
        h, w = img.shape[0], img.shape[1]
        if (self.transformMatrix is None):
            print("before warp call get_transform_matrix()")
        else:
            self.birds_image = cv2.warpPerspective(np.copy(img), self.transformMatrix, (w, h))            
        return self.birds_image

    # Gradient and color tresholds
    def treshold_binary(self, image, s_thresh=(200, 255), sx_thresh=(20, 100)):
        # Convert to HLS color space and separate the V channel
        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        l_channel = hls[:,:,1]
        s_channel = hls[:,:,2]
         
        # Sobel x
        sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 0) # Take the derivative in x
        abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
        scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx)) if (np.max(abs_sobelx)) else np.uint8(255*abs_sobelx)
        
        # Threshold x gradient
        sxbinary = np.zeros_like(scaled_sobel)
        sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
        
        # Threshold color channel
        # in simulation h and s channels are zeros
        if (np.max(s_channel) == 0):
            s_channel = l_channel
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
        
        # Combine the two binary thresholds
        combined_binary = np.zeros_like(sxbinary)
        combined_binary[(s_binary == 1) | (sxbinary == 1)] = 1

        binary = np.dstack((combined_binary, combined_binary, combined_binary)) * 255
        self.treshold_image = binary

        return binary

    # get middle line points from left and right lane polynomials
    def get_waypoints(self):        
        # if the image is empty - return a point straight ahead from the robocar
        if (np.count_nonzero(self.left_fit) == np.count_nonzero(self.right_fit) == 0):
            return np.array([[1, 0, 0]])
        
        ploty, left_fitx, right_fitx = self.get_xy(10)
        
        middle_fitx = (left_fitx + right_fitx) / 2
        # print(middle_fitx)
        # center_array = np.column_stack((np.flip(middle_fitx, 0), ploty, np.zeros_like(ploty))
        # print(center_array)
        # substract img.width/2 to find deviation from center line
        # center_array[:, 0]-= int(self.shape[1] / 2)
        middle_fitx -= int(self.shape[1] / 2)
        # print(center_array)
        # multiple by scaling parameters in x and y directions
        # to find coodrinates in required units and required sign
        # center_array[:, 0]/= -1
        middle_fitx *= -self.scale[0]
        ploty *= self.scale[1]
        # print(center_array)
        # center_array[:,[0, 1]] = center_array[:,[1, 0]]
        center_array = np.column_stack((ploty, np.flip(middle_fitx, 0), np.zeros_like(ploty)))        
        # print(center_array[1:])
        self.center_array = center_array

        return center_array

    # return polynomial for both lanes
    # ym_per_pix, xm_per_pix - meters per pixel in x or y dimension
    def fit_polynomial(self, treshold_warped, draw=False, ym_per_pix=1, xm_per_pix=1):
        self.shape = treshold_warped.shape
        self.poly_img = np.copy(treshold_warped)
        # Find our lane pixels first
        leftx, lefty, rightx, righty = self.find_lane_pixels(draw)
        left_fit = [0,0,0]
        right_fit = [0,0,0]
        
        
        # check if the image is not empty
        if (leftx.size > 0 and lefty.size > 0 and rightx.size > 0 and righty.size > 0):
            left_fit = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
            right_fit = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)      
        self.left_fit = left_fit
        self.right_fit = right_fit

        return self.left_fit, self.right_fit, self.poly_img

    # find lane pixels in image with sliding windows
    def find_lane_pixels(self, draw=False):
        treshold_warped = self.poly_img[:,:,0]
        # Take a histogram of the bottom half of the image
        histogram = np.sum(treshold_warped[treshold_warped.shape[0]//2:,:], axis=0)
        # if the image is empty
        if (np.amax(histogram) < 10):
            leftx = lefty = rightx = righty = np.array([])
        else:        
            # Find the peak of the left and right halves of the histogram
            # These will be the starting point for the left and right lines
            midpoint = np.int(histogram.shape[0]//2)
            leftx_base = np.argmax(histogram[:midpoint])
            rightx_base = np.argmax(histogram[midpoint:]) + midpoint        

            # Set height of windows - based on nwindows above and image shape
            window_height = np.int(treshold_warped.shape[0]//self.nwindows)
            # Identify the x and y positions of all nonzero pixels in the image
            nonzero = treshold_warped.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            # Current positions to be updated later for each window in nwindows
            leftx_current = leftx_base
            rightx_current = rightx_base

            # Create empty lists to receive left and right lane pixel indices
            left_lane_inds = []
            right_lane_inds = []

            # stop flag if there are no lane pixels
            left_stop = False
            right_stop = False

            # Step through the windows one by one
            for window in range(self.nwindows):
                # Identify window boundaries in x and y (and right and left)
                win_y_low = treshold_warped.shape[0] - (window+1)*window_height
                win_y_high = treshold_warped.shape[0] - window*window_height
                
                if not left_stop:
                    # Find the four below boundaries of the window 
                    win_xleft_low = leftx_current - self.margin 
                    win_xleft_high = leftx_current + self.margin                     
                    # draw window rectangles
                    if draw:
                        cv2.rectangle(self.poly_img,(win_xleft_low,win_y_low),
                        (win_xleft_high,win_y_high),(0,255,0), 2) 
                    # Identify the nonzero pixels in x and y within the window 
                    good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                    (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
                    # Append these indices to the lists
                    left_lane_inds.append(good_left_inds)
                    # If you found > minpix pixels, recenter next window on their mean position 
                    # or stop windows
                    if len(good_left_inds) > self.minpix:
                        leftx_current=np.int(np.mean(nonzerox[good_left_inds]))
                    else:
                        left_stop = True

                if not right_stop:
                    # Find the four below boundaries of the window 
                    win_xright_low = rightx_current - self.margin 
                    win_xright_high = rightx_current + self.margin                      
                    # draw window rectangles
                    if draw:
                        cv2.rectangle(self.poly_img,(win_xright_low,win_y_low),
                        (win_xright_high,win_y_high),(0,255,0), 2)  

                    # Identify the nonzero pixels in x and y within the window                     
                    good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                    (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
                    # Append these indices to the lists
                    right_lane_inds.append(good_right_inds)
                    # If you found > minpix pixels, recenter next window on their mean position 
                    # or stop windows
                    if (good_right_inds.shape[0] > self.minpix):
                        rightx_current=np.int(np.mean(nonzerox[good_right_inds]))
                    else:
                        right_stop = True

            # Concatenate the arrays of indices (previously was a list of lists of pixels)
            try:
                left_lane_inds = np.concatenate(left_lane_inds)
                right_lane_inds = np.concatenate(right_lane_inds)
            except ValueError:
                # Avoids an error if the above is not implemented fully
                pass

            # Extract left and right line pixel positions
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds] 
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]
    
        return leftx, lefty, rightx, righty

    # draw a green polygon between two lanes
    def draw_filled_polygon(self):
        ploty, left_fitx, right_fitx = self.get_xy(self.shape[0])
        
        all_x = np.concatenate([left_fitx, np.flip(right_fitx, 0)])
        all_y = np.concatenate([ploty, np.flip(ploty, 0)])
        all_points = [(np.asarray([all_x, all_y]).T).astype(np.int32)]
        cv2.fillPoly(self.poly_img, all_points, (0,255,0))
        
        return self.poly_img
    
    def get_xy(self, num_points):
        ploty = np.linspace(0, self.shape[0]-1, num_points)
        try:
            left_fitx = self.left_fit[0]*ploty**2 + self.left_fit[1]*ploty + self.left_fit[2]
            right_fitx = self.right_fit[0]*ploty**2 + self.right_fit[1]*ploty + self.right_fit[2]
        except TypeError:
            # Avoids an error if `left` and `right_fit` are still none or incorrect
            print('The function failed to fit a line!')
            left_fitx = 1*ploty**2 + 1*ploty
            right_fitx = 1*ploty**2 + 1*ploty
            
        return ploty, left_fitx, right_fitx