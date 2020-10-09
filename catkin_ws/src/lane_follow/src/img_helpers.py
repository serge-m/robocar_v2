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
       
        self.birds_size = (int(s*W[0]), int(s*W[1]))
        self.birds_image = np.zeros(self.birds_size)

    def warpPerspective(self, img):
        self.birds_image = cv2.warpPerspective(img, self.transform, self.birds_size, flags=
				cv2.INTER_LINEAR+cv2.WARP_FILL_OUTLIERS+cv2.WARP_INVERSE_MAP)

# class for treshold functions
class Treshold:
    def __init__(self):
        self.image = None
    # Gradient and color tresholds
    def treshold_binary(self, image, s_thresh=(200, 255), sx_thresh=(20, 90)):
        # Convert to HLS color space and separate the V channel
        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        l_channel = hls[:,:,1]
        s_channel = hls[:,:,2]
        # Sobel x
        sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 0, ksize=7) # Take the derivative in x
        abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
        scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx)) if (np.max(abs_sobelx)) else np.uint8(255*abs_sobelx)
        
        # Threshold x gradient
        sxbinary = np.zeros_like(scaled_sobel)
        sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
        
        # Threshold color channel
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
        
        # Combine the two binary thresholds
        combined_binary = np.zeros_like(sxbinary)
        combined_binary[(s_binary == 1) | (sxbinary == 1)] = 1

        binary = np.dstack((combined_binary, combined_binary, combined_binary)) * 255
        self.image = binary

        return binary

# class for polynomial functions
class FitPolynomial:
    def __init__(self, nwindows=8, margin=100, minpix=50):
        self.left_fit = None
        self.right_fit = None
        self.center_array = None
        # HYPERPARAMETERS
        # Choose the number of sliding windows
        self.nwindows = nwindows
        # Set the width of the windows +/- margin
        self.margin = margin
        # Set minimum number of pixels found to recenter window
        self.minpix = minpix

    # get middle line points from left and right lane polynomials
    def get_waypoints(self, shape, M):
        ploty = np.linspace(0, shape[0]-1, shape[0])
        # if the image is empty - return a point straight ahead from the robocar
        if (np.count_nonzero(self.left_fit) == np.count_nonzero(self.right_fit) == 0):
            return np.array([[1, 0]])
        try:
            left_fitx = self.left_fit[0]*ploty**2 + self.left_fit[1]*ploty + self.left_fit[2]
            right_fitx = self.right_fit[0]*ploty**2 + self.right_fit[1]*ploty + self.right_fit[2]
        except TypeError:
            # Avoids an error if `left` and `right_fit` are still none or incorrect
            left_fitx = 1*ploty**2 + 1*ploty
            right_fitx = 1*ploty**2 + 1*ploty
        middle_fitx = (left_fitx + right_fitx) / 2
        # print(middle_fitx)
        center_array = np.column_stack((middle_fitx, np.flip(ploty, 0)))
        # print(center_array)
        # substract img.width/2 to find deviation from center line
        center_array[:, 0]-= int(shape[1] / 2)
        # print(center_array)
        # divide by scaling parameters in x and y directions
        # to find coodrinates in required units and required sign
        center_array[:, 0]/= -M[0]
        # center_array[:, 0]/= M[0]
        center_array[:, 1]/= M[1]
        # print(center_array)
        center_array[:,[0, 1]] = center_array[:,[1, 0]]
        # print(center_array[0])
        self.center_array = center_array

        return center_array

    # return polynomial for both lanes
    # ym_per_pix, xm_per_pix - meters per pixel in x or y dimension
    def fit_polynomial(self, treshold_warped, ym_per_pix=1, xm_per_pix=1):
        # Find our lane pixels first
        leftx, lefty, rightx, righty = self.find_lane_pixels(treshold_warped)
        left_fit = [0,0,0]
        right_fit = [0,0,0]
        
        # check if the image is not empty
        if (leftx.size > 0 and lefty.size > 0 and rightx.size > 0 and righty.size > 0):
            left_fit = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
            right_fit = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)      

        self.left_fit = left_fit
        self.right_fit = right_fit

    # find lane pixels in image with sliding windows
    def find_lane_pixels(self, img):
        treshold_warped = img[:,:,0]
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

            # Step through the windows one by one
            for window in range(self.nwindows):
                # Identify window boundaries in x and y (and right and left)
                win_y_low = treshold_warped.shape[0] - (window+1)*window_height
                win_y_high = treshold_warped.shape[0] - window*window_height
                # Find the four below boundaries of the window 
                win_xleft_low = leftx_current - self.margin  # Update this
                win_xleft_high = leftx_current + self.margin  # Update this
                win_xright_low = rightx_current - self.margin  # Update this
                win_xright_high = rightx_current + self.margin  # Update this       
                
                # Identify the nonzero pixels in x and y within the window 
                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
                # Append these indices to the lists
                left_lane_inds.append(good_left_inds)
                right_lane_inds.append(good_right_inds)
                # If you found > minpix pixels, recenter next window 
                # (`right` or `leftx_current`) on their mean position 
                if len(good_left_inds) > self.minpix:
                    leftx_current=np.int(np.mean(nonzerox[good_left_inds]))
                if (good_right_inds.shape[0] > self.minpix):
                    rightx_current=np.int(np.mean(nonzerox[good_right_inds]))

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