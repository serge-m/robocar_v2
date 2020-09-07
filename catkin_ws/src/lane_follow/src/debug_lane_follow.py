import sys, time
import cv2
from lane_follower import LaneFollower

def main(args):
    camera_params = '../../robocar_config/camera_calibration.yaml'    
    lane_follower = LaneFollower(camera_params, resize_if_needed=True)
    image_np = cv2.imread('C:/tmp/img/2121.305.jpg')
    debug_output = {}
    desired_shift = lane_follower(image_np, debug_output)
    
if __name__ == '__main__':
    main(sys.argv) 