#!/usr/bin/env python
from __future__ import print_function, division
import numpy as np

class LaneFollower:
    def __init__(self):
        pass
    
    def __call__(self, image: np.ndarray) -> np.ndarray:
        image = image.copy()
        image[:image.shape[0] // 2] = 0
        return image
