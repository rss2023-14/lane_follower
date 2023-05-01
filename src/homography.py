#!/usr/bin/env python

import rospy
import numpy as np
import cv2

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

##############################################
# pixels (i,j), images taken from left camera sensor
PTS_IMAGE_PLANE = [[223, 321],
                   [366, 294],
                   [515, 328],
                   [155, 274],
                   [285, 260],
                   [379, 246],
                   [524, 253],
                   [300, 218],
                   [388, 219]]

# inches (x,y), where x is forward and y is left of left camera sensor
PTS_GROUND_PLANE = [[15.5,   4.0],
                    [17.75, -3.5],
                    [13.0,  -9.5],
                    [22.0,   10.75],
                    [24.5,   2.0],
                    [28.0,  -5.5],
                    [24.5,  -16.0],
                    [41.75,  2.25],
                    [39.5,  -8.0]]
##############################################

class HomographyTransformer:
    """
    Takes lanes detected and returns the best lookahead point for the pure
    pursuit controller to follow.

    Responsible for homography transforms from image in left camera frame to
    world in base_link frame.
    """

    def __init__(self):
        # Initialize homography matrix
        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rospy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    def find_lookahead_point(self):

        def line_equation(line, x):
            y = line[0] * x + line[1]
            return y
        
        def midpoint_formula(x1, y1, x2, y2):
            x = (x1 + x2)/2
            y = (y1 + y2)/2
            return (x, y)

        self.LOOKAHEAD_HOMOGRAPHY = rospy.get_param("lookahead_distance", 1.0) # can change rosparam here
        # subscribe to topic that is publishing the lines in the in lane detector
        # lines are in the form [(x_lane_1,y_return),(x_lane_2,y_return)] (x, y)
        line_one, line_two = returnObject

        if line_two == None: # only left line detected

            pass
        elif line_one == None: # only right line detected

            pass
        else: # both lines
            
            pass


    def pixel_to_world(self, i, j):
        """
        Takes pixel coordinates (i,j) and returns world frame coordinates (x,y) in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return (x, y)

    def transform_to_base_link(self):
        pass

    def publish_lookahead_point(self):
        pass


if __name__ == "__main__":
    rospy.init_node("homography")
    ht = HomographyTransformer()
    rospy.spin()
