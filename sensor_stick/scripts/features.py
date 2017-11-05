
#!/usr/bin/env python
"""
Module for computing color histograms to obtain features for identifying objects.

The objective is to obtain some set of features like color, shape, size etc. that 
can be used to identify a particular object in a segmented point cloud. 

For this purpose, a color histogram is chosen to convert color information 
into features. A classifier can then be trained on this feature set to
recognize the object of interest in the point cloud. Compared to RGB, the HSV
color space is a more robust color space for perception tasks in robotics as it 
is resistant to lighting changes.

This technique is useful as it removes any dependancy on spatial structure
i.e. objects that are in slightly different poses and orientations will still
be matched. Further, the color histogram can be normalized to account for
variations in object size. Note, however, that this technique can sometimes 
match some unwanted regions resulting in false positives.

"""

__author__ = 'Salman Hashmi'


import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from pcl_helper import *


def rgb_to_hsv(rgb_list):
    """Convert a list of RGB colors into HSV colors."""
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized

