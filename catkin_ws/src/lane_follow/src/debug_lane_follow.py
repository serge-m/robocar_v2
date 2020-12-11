import sys, time
import os
import cv2
import math
import numpy as np
from img_helpers import ImageProcessor
from lane_follower import LaneFollower

def main(args):
    test_dir_path = os.path.join(os.path.split(os.path.dirname(__file__))[0], 'test')
    fname = os.path.join(test_dir_path, '1.jpg')
    image_np = cv2.imread(fname)  
    h, w = image_np.shape[0], image_np.shape[1] 
    image_proc = ImageProcessor()
    # testing binary tresholding
    treshold_image = image_proc.treshold_binary(image_np)
    cv2.imwrite(os.path.join(test_dir_path, 'treshold_image.jpg'), treshold_image)
    
    # testing warp functions    
    src = np.float32([[387, 525],[894, 525],[w, h],[0, h]])
    dst = np.float32([[0, 0],[w, 0],[w, h],[0, h]])
    image_proc.get_transform_matrix(src, dst)
    x_scale = 1.7
    y_scale = 2.
    image_proc.setScale((x_scale/w, y_scale/h))
    birds_image = image_proc.warp_perspective(treshold_image)
    cv2.imwrite(os.path.join(test_dir_path, 'birds_image.jpg'), birds_image)
    
    # testing fitting polynomial
    left_fit, right_fit, poly_img = image_proc.fit_polynomial(birds_image, True)
    print("left fit", str(left_fit))
    print("right fit", str(right_fit))
    cv2.imwrite(os.path.join(test_dir_path, 'poly.jpg'), poly_img)
    filled_img = image_proc.draw_filled_polygon()
    cv2.imwrite(os.path.join(test_dir_path, 'filled_poly.jpg'), filled_img)
    
    # testing getting new waypoints
    waypoints = image_proc.get_waypoints()
    print("waypoints", waypoints)

    # testing whole laneFollower
    image_np = cv2.imread(fname)
    lf = LaneFollower()
    src = np.float32([[387, 525],[894, 525],[w, h],[0, h]])
    dst = np.float32([[0, 0],[w, 0],[w, h],[0, h]])
    lf.image_proc.get_transform_matrix(src, dst)
    x_scale = 1.7
    y_scale = 2.
    lf.image_proc.setScale((x_scale/w, y_scale/h))
    lf_waypoints = lf(image_np)
    print("lf_waypoints", lf_waypoints)
    
    
if __name__ == '__main__':
    main(sys.argv) 