#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import tf2_ros

from geometry_msgs.msg import PointStamped, TransformStamped, PoseArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import tf2_geometry_msgs

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

METERS_PER_INCH = 0.0254
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

        # Prepare for transforms between base_link and left_zed_camera
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # set offset for instances where only one line was foud
        self.SINGLE_LANE_OFFSET = rospy.get_param("single_lane_offset", 5.0) # can change rosparam here

        # Take lane messages, publish lookahead points
        LOOKAHEAD_TOPIC = rospy.get_param("lookahead_topic")
        LANE_TOPIC = rospy.get_param("lane_topic")
        self.lookahead_pub = rospy.Publisher(LOOKAHEAD_TOPIC, PointStamped, queue_size=1)
        self.lane_sub = rospy.Subscriber(LANE_TOPIC, PoseArray, self.find_lookahead_point, queue_size=1) # TODO: Change message type

    def find_lookahead_point(self, msg):
        """
        using points from two lines, return a point for the car to go 
        to in the pixel frame 
        """

        def line_equation(line, x):
            y = line[0] * x + line[1]
            return y
        
        def midpoint_formula(x1, y1, x2, y2):
            x = (x1 + x2)/2
            y = (y1 + y2)/2
            return (x, y)
    
        line_one, line_two = msg # msg should be a 2 element array of lines in the form [(x_lane_1,y_return),(x_lane_2,y_return)] (x, y)
        
        if line_two is None: # only left line detected, set distance, will select based on axis
            x1 = line_one.position.x + self.SINGLE_LANE_OFFSET
            y1 = line_one.position.y
            line_one_real_world = self.pixel_to_world(x1, y1)
            
            to_chase = PointStamped()
            to_chase.header = Header()
            to_chase.header.frame_id = "frame_id" # TODO
            to_chase.point.x = line_one_real_world[0]
            to_chase.point.y = line_one_real_world[1]

            
        elif line_one is None: # only right line detected
            x2 = line_two.position.x - self.SINGLE_LANE_OFFSET
            y2 = line_two.position.y
            line_two_real_world = self.pixel_to_world(x2, y2)
            
            to_chase = PointStamped()
            to_chase.header = Header()
            to_chase.header.frame_id = "frame_id" # TODO
            to_chase.point.x = line_two_real_world[0]
            to_chase.point.y = line_two_real_world[1]
            
            
        else: # both lines, mean
            x1 = line_one.position.x
            y1 = line_one.position.y
            x2 = line_two.position.x
            y2 = line_two.position.y
            line_one_real_world = self.pixel_to_world(x1, y1)
            line_two_real_world = self.pixel_to_world(x2, y2)
            midpoint = midpoint_formula(line_one_real_world[0], line_one_real_world[1], line_two_real_world[0], line_two_real_world[1])
            
            to_chase = PointStamped()
            to_chase.header = Header()
            to_chase.header.frame_id = "frame_id" # TODO
            to_chase.point.x = midpoint[0]
            to_chase.point.y = midpoint[1]

        to_return = self.transform_to_base_link(to_chase)
        self.lookahead_pub.publish(to_return)
        

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

    def transform_to_base_link(self, point):
        """
        Takes a PointStamped message and transforms it from the its frame_id frame
        into the base_link frame.
        """
        try:
            point_transformed = self.tf_buffer.transform(point, "base_link", rospy.Duration(0.1))
            return point_transformed
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise


if __name__ == "__main__":
    rospy.init_node("homography")
    ht = HomographyTransformer()
    rospy.spin()
