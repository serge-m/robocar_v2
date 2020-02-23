#!/usr/bin/env python
from __future__ import print_function, division
import numpy as np
from typing import Dict, Optional

class LaneFollower:
    def __init__(self):
        pass
    
    def __call__(self, image: np.ndarray, debug_output: Optional[Dict[str, object]]=None) -> np.ndarray:
        if debug_output is not None:
            vis = image.copy()
            vis[:image.shape[0] // 2] = vis[:image.shape[0] // 2] // 2
            debug_output['vis'] = vis
        return image.ravel()[:3]
