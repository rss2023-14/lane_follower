#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import tf2_ros

from geometry_msgs.msg import PointStamped, TransformStamped
from lane_follower.msg import Lane
from visualization_msgs.msg import Marker
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

        # Set offset for instances where only one line was found
        self.LEFT_LANE_OFFSET = rospy.get_param("left_lane_offset")
        self.RIGHT_LANE_OFFSET = rospy.get_param("right_lane_offset")
        # Distance ahead of car we should find a lookahead point
        self.LOOKAHEAD_DISTANCE = rospy.get_param("lookahead_distance")

        # Take lane messages, publish lookahead points
        LOOKAHEAD_TOPIC = rospy.get_param("lookahead_topic")
        LANE_TOPIC = rospy.get_param("lane_topic")
        self.lookahead_pub = rospy.Publisher(LOOKAHEAD_TOPIC, PointStamped, queue_size=1)
        self.lane_sub = rospy.Subscriber(LANE_TOPIC, Lane, self.find_lookahead_point, queue_size=1)

    def find_lookahead_point(self, msg):
        """
        Using points from two lines, return a point for the car to chase in the world frame.
        """
        def midpoint_formula(x1, y1, x2, y2):
            x = (x1 + x2)/2
            y = (y1 + y2)/2
            return (x, y)

        to_chase = PointStamped()
        to_chase.header.stamp = rospy.Time.now()
        to_chase.header.frame_id = "left_zed_camera"    
        if msg.detectedLeft and msg.detectedRight:
            # Both lines detected
            x1 = msg.lineLeft.x
            y1 = msg.lineLeft.y
            x2 = msg.lineRight.x
            y2 = msg.lineRight.y
            lineLeft_world = self.pixel_to_world(x1, y1)
            lineRight_world = self.pixel_to_world(x2, y2)
            midpoint = midpoint_formula(lineLeft_world[0], lineLeft_world[1], lineRight_world[0], lineRight_world[1])

            to_chase.point.x = midpoint[0]
            to_chase.point.y = midpoint[1]
        elif msg.detectedLeft:
            # Only left line detected
            x1 = min(msg.lineLeft.x+self.LEFT_LANE_OFFSET, 650)
            y1 = msg.lineLeft.y
            lineLeft_world = self.pixel_to_world(x1, y1)

            to_chase.point.x = lineLeft_world[0]
            to_chase.point.y = lineLeft_world[1]
        elif msg.detectedRight:
            # Only right line detected
            x2 = max(msg.lineRight.x-self.RIGHT_LANE_OFFSET, 25)
            y2 = msg.lineRight.y
            lineRight_world = self.pixel_to_world(x2, y2)

            to_chase.point.x = lineRight_world[0]
            to_chase.point.y = lineRight_world[1]
        else:
            # No lines detected, publish point right in front of car
            to_chase.point.x = self.LOOKAHEAD_DISTANCE
            to_chase.point.y = 0.0

        """
        DEBUGGING, pixel in direct middle of image
        """
        # x, y = self.pixel_to_world(672/2.0 + 50, 0.8*376)
        # to_chase.point.x = x
        # to_chase.point.y = y

        to_return = self.transform_to_car(to_chase)
        to_return.point.x = self.LOOKAHEAD_DISTANCE
        self.lookahead_pub.publish(to_return)

    def pixel_to_world(self, u, v):
        """
        Takes pixel coordinates (u,v) and returns world frame coordinates (x,y) in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return (x, y)

    def transform_to_car(self, point):
        """
        Takes a PointStamped message and transforms it from the its frame_id frame
        into the laser frame, which is in front of the car.
        """
        try:
            point_transformed = self.tf_buffer.transform(point, "laser", rospy.Duration(0.1))
            return point_transformed
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise


if __name__ == "__main__":
    rospy.init_node("homography")
    ht = HomographyTransformer()
    rospy.spin()
