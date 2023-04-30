#!/usr/bin/env python

import rospy
import numpy as np
import cv2

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class HomographyTransformer:
    """
    Takes lanes detected and returns the best lookahead point for the pure
    pursuit controller to follow.

    Responsible for homography transforms from image in left camera frame to
    world in base_link frame.
    """

    def __init__(self):
        pass

    def find_lookahead_point(self):
        pass

    def pixel_to_world(self):
        pass

    def transform_to_base_link(self):
        pass

    def publish_lookahead_point(self):
        pass


if __name__ == "__main__":
    rospy.init_node("homography")
    ht = HomographyTransformer()
    rospy.spin()
