#!/usr/bin/env python
from __future__ import print_function, division
import numpy as np
# from typing import Dict, Optional
import yaml
import cv2
import rospy

def _load_camera_params(path):
    with open(path) as f:
        cam_yaml = yaml.safe_load(f)
    result = {
        'image_width': cam_yaml['image_width'],
        'image_height': cam_yaml['image_height'],
        'camera_matrix': np.array(cam_yaml['camera_matrix']['data']).reshape((3,3)),
        'distortion_coefficients': np.array(cam_yaml['distortion_coefficients']['data']).reshape((5))
    }
    result['new_camera_matrix'], result['valid_roi'] = cv2.getOptimalNewCameraMatrix(
        result['camera_matrix'], 
        result['distortion_coefficients'], 
        imageSize=(result['image_height'], result['image_width']), 
        alpha=1.)
    return result

class LaneFollower:
    # (self, file_camera_params: str=None, resize_if_needed=False)
    def __init__(self, file_camera_params=None, resize_if_needed=False):
        self.camera_params = _load_camera_params(file_camera_params) if file_camera_params is not None else None
        self.resize_if_needed = resize_if_needed

#    (self, image: np.ndarray, debug_output: Optional[Dict[str, object]]=None) -> np.ndarray 
    def __call__(self, image, debug_output=None):
        waypoints = [[1, 0]]
        if self.camera_params is not None:
             # Undistort image
            image_undist = undistort(image, self.camera_params, self.resize_if_needed)
            
            
            # Make gradient and color treshold
            treshold = treshold_binary(image_undist)
            # get matrices for perspective transform
            perspective_M, Minv = get_warp_matrices(image_undist, self.camera_params['camera_matrix'])
            # Warp tresholded image
            warp_img = unwarp(treshold, perspective_M)
            warp_save = np.dstack((warp_img, warp_img, warp_img)) * 255
            
            # Calculate polynomials for left and right lanes
            left_fit, right_fit, out_img = fit_polynomial(warp_img)
            predicted_waypoints = get_waypoints(image.shape, left_fit, right_fit, self.camera_params['camera_matrix'])
            if (predicted_waypoints.size > 0): 
                waypoints = predicted_waypoints
                # print(waypoints[0])
                # rospy.loginfo("waypoints are %s", [0.5, -waypoints[0][1]])
                rospy.loginfo("waypoints are %s", waypoints[0])
                time = str(rospy.get_time()) + str(waypoints[0])
                # cv2.imwrite('C:/tmp/img/' + time + '.jpg', image_undist)
                # cv2.imwrite('C:/tmp/img/warp_' + time + '.jpg', warp_save)
        else:
            image_undist = image
        if debug_output is not None:
            debug_output['vis'] = image_undist
        return waypoints[0]

# (image: np.ndarray, camera_params: Dict, resize_if_needed: bool) -> np.ndarray
def undistort(image, camera_params, resize_if_needed):
    w_h_actual = (image.shape[1], image.shape[0])
    w_h_expected = (camera_params['image_width'], camera_params['image_height'])
    
    if w_h_actual != w_h_expected:
        if resize_if_needed:
            image = cv2.resize(image, w_h_expected, interpolation=cv2.INTER_LINEAR)
        else:
            raise RuntimeError("Camera calibration is set but the size of input doesn't match. "
                        "expected: {}, actual: {}".format(w_h_expected, w_h_actual))

    return cv2.undistort(
        image, camera_params['camera_matrix'], camera_params['distortion_coefficients'], 
        newCameraMatrix=camera_params['new_camera_matrix']
    )

# get middle line points from left and right lane polynomials
def get_waypoints(shape, left_fit, right_fit, M):
    ploty = np.linspace(0, shape[0]-1, shape[0])
    try:
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    except TypeError:
        # Avoids an error if `left` and `right_fit` are still none or incorrect
        # rospy.loginfo('The function failed to fit a line!')
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
    center_array[:, 0]/= -M[0][0]
    # center_array[:, 0]/= M[0][0]
    center_array[:, 1]/= M[1][1]
    # print(center_array)
    center_array[:,[0, 1]] = center_array[:,[1, 0]]
    # print(center_array[0])
   
    return center_array

# Gradient and color tresholds
def treshold_binary(image, s_thresh=(200, 255), sx_thresh=(20, 90)):
    img = np.copy(image)
    # Convert to HLS color space and separate the V channel
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
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
    
    return combined_binary

# get Matrix and Inverse Matrix for Perspective Transform
def get_warp_matrices(image, M):
    w = image.shape[1] # 1280
    h = image.shape[0] # 960
    # get transform points
    # TODO points should be dependant on height and angle of camera, not hard-corded
    src = np.float32([[0, h-150],[555, 520],[w-530, 520],[w, h-150]])
    # scaling parameters in x and y directions
    # road width will be equal to one unit length
    x = M[0][0]
    y = M[1][1]

    dst = np.float32([[int((w-x)/2), h],
                    [int((w-x)/2), int((h-y))],
                    [int((w+x)/2), int((h-y))],
                    [int((w+x)/2), h]])
    
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    return  M, Minv

# Apply a perspective transform to an image
def unwarp(img, M):
    img_size = (img.shape[1], img.shape[0])
    warped = cv2.warpPerspective(np.copy(img), M, img_size, flags=cv2.INTER_LINEAR)
    
    return warped

# return polynomial for both lanes
# ym_per_pix, xm_per_pix - meters per pixel in x or y dimension
def fit_polynomial(binary_warped, ym_per_pix=1, xm_per_pix=1):
    # Find our lane pixels first
    leftx, lefty, rightx, righty, out_img = find_lane_pixels(binary_warped)
    left_fit = [0,0,0]
    right_fit = [0,0,0]
    if (leftx.size & lefty.size & rightx.size & righty.size):        
        left_fit = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
        right_fit = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)      
    
    return left_fit, right_fit, out_img

# find lane pixels in image with sliding windows
def find_lane_pixels(binary_warped):
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
    # Create an output image to draw on and visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0]//2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # HYPERPARAMETERS
    # Choose the number of sliding windows
    nwindows = 8
    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 50

    # Set height of windows - based on nwindows above and image shape
    window_height = np.int(binary_warped.shape[0]//nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated later for each window in nwindows
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        # Find the four below boundaries of the window 
        win_xleft_low = leftx_current - margin  # Update this
        win_xleft_high = leftx_current + margin  # Update this
        win_xright_low = rightx_current - margin  # Update this
        win_xright_high = rightx_current + margin  # Update this       
        
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
        if len(good_left_inds) > minpix:
            leftx_current=np.int(np.mean(nonzerox[good_left_inds]))
        if (good_right_inds.shape[0] > minpix):
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
 
    return leftx, lefty, rightx, righty, out_img
