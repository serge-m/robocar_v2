#!/usr/bin/env python
from __future__ import print_function, division
import numpy as np
from typing import Dict, Optional
import yaml
import cv2

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
    def __init__(self, file_camera_params: str=None):
        self.camera_params = _load_camera_params(file_camera_params) if file_camera_params is not None else None
           
    def __call__(self, image: np.ndarray, debug_output: Optional[Dict[str, object]]=None) -> np.ndarray:
        if self.camera_params is not None:
            image_undist = undistort(image, self.camera_params)
        else:
            image_undist = image

        if debug_output is not None:
            debug_output['vis'] = image_undist
        return image.ravel()[:3]

def undistort(image: np.ndarray, camera_params: Dict) -> np.ndarray:
    h_w_actual = (image.shape[0], image.shape[1])
    h_w_expected = (camera_params['image_height'], camera_params['image_width'])
    if h_w_actual != h_w_expected:
        raise RuntimeError("Camera calibration is set but the size of input doesn't match. "
                        "expected: {}, actual: {}".format(h_w_expected, h_w_actual))
    return cv2.undistort(
        image, camera_params['camera_matrix'], camera_params['distortion_coefficients'], 
        newCameraMatrix=camera_params['new_camera_matrix']
    )