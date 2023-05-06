#!/usr/bin/env python

import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import copy
# from skimage.transform import downscale_local_mean

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from lane_follower.msg import Lane


class LaneDetector:
    """
    Take rgb camera input from zed sensor, and attempt to find left and right line corresponding
    to lane. Publishes Lane msg with this info.
    """

    def __init__(self):
        self.LOOKAHEAD_HOMOGRAPHY = rospy.get_param("lookahead_distance_homog", 0.9)

        LANE_TOPIC = rospy.get_param("lane_topic")
        self.lane_pub = rospy.Publisher(LANE_TOPIC, Lane, queue_size=1)

        # For zed camera
        self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber(
            "/zed/zed_node/rgb/image_rect_color", Image, self.detect_lane
        )

    def detect_lane(self, image):
        # img = downscale_local_mean(self.bridge.imgmsg_to_cv2(image, "bgr8"), (2, 2))
        img = self.bridge.imgmsg_to_cv2(image, "bgr8")

        # Mask top half of image
        top_half_mask = np.zeros_like(img)
        height = img.shape[0]
        height_part = int(np.floor(height * 0.5))
        top_half_mask[height_part:, :] = 255
        blacked_img = cv.bitwise_and(img, top_half_mask)

        # Grayscale and blur image to get the edges
        gray_img = cv.cvtColor(blacked_img, cv.COLOR_BGR2GRAY)
        white_thresh = cv.threshold(gray_img, 150, 255, cv.THRESH_BINARY)[1]
        blur = cv.GaussianBlur(white_thresh, (5, 5), 0)
        edges = cv.Canny(blur, 50, 150, apertureSize=3)

        # Get the lines from opencv
        lines = cv.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=50,
            minLineLength=150,
            maxLineGap=10,
        )

        msg = Lane()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "left_zed_camera"

        # Check that there are lines found
        if lines is None:
            rospy.logwarn("No lines found!")
            self.lane_pub.publish(msg)
            return

        # Filter lines based on their angle, aiming for almost vertical lines
        filtered_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1)
            # angle = np.arctan2(y2-y1, x2-x1) * 180 / np.pi
            if abs(slope) < 0.25:
                continue
            # if abs(angle) < 30: # vertical lines
            elif abs(slope) > 0.65: 
                continue
            else:
                filtered_lines.append(list(line[0]))

        """
        Filter lines again, this time to try to get a single line per lane.
        Basically, I am checking all the lines to see if there are lines which have very similar slopes, and if so,
        I am deleting the ones which have similar slopes and are further from the center of the image.
        """
        slope_tracker = {}
        copy_filt = copy.deepcopy(filtered_lines)
        for line in copy_filt:
            x1, y1, x2, y2 = line
            slope = (y2 - y1) / (x2 - x1)
            get_rid = False
            if slope_tracker != {}:
                copy_slopes = copy.deepcopy(slope_tracker)
                for oth_slope in copy_slopes.keys():
                    if abs(slope - oth_slope) < 0.20:
                        if slope < 0:
                            if x2 < slope_tracker[oth_slope][2]:
                                filtered_lines.remove(slope_tracker[oth_slope])
                                del slope_tracker[oth_slope]
                            else:
                                get_rid = True
                                break
                        else:
                            if x2 > slope_tracker[oth_slope][2]:
                                filtered_lines.remove(slope_tracker[oth_slope])
                                del slope_tracker[oth_slope]
                            else:
                                get_rid = True
                                break
            if get_rid:
                filtered_lines.remove(line)
                continue
            slope_tracker[slope] = line

        """
        # This is mostly for visualization on the images, won't be needed in the actual function,
        # however, I do construct the line equations here, which we may want to do pre homography.
        print(slope_tracker)
        """
        rospy.loginfo(str(len(filtered_lines)) + " lines found.")

        y_return = int(np.floor(height * self.LOOKAHEAD_HOMOGRAPHY))
        if len(filtered_lines) == 2:
            # Publish both lines
            x1, y1, x2, y2 = filtered_lines[0]
            m_1 = float(y2 - y1) / (x2 - x1)
            b = y1 - m_1 * x1
            x_lane_1 = (y_return - b) / m_1

            x1, y1, x2, y2 = filtered_lines[1]
            m_2 = float(y2 - y1) / (x2 - x1)
            b = y1 - m_2 * x1
            x_lane_2 = (y_return - b) / m_2

            # x_return = int(np.floor((x_lane_1+x_lane_2)/2))
            # cv.circle(img, (x_return,y_return), 5, (0, 0, 255), -1)

            p1 = Point()
            p1.x = x_lane_1
            p1.y = y_return
            p2 = Point()
            p2.x = x_lane_2
            p2.y = y_return

            msg.detectedLeft = True
            msg.detectedRight = True
            if m_1 < 0:
                msg.lineLeft = p1
                msg.lineRight = p2
            else:
                msg.lineLeft = p2
                msg.lineRight = p1

        elif len(filtered_lines) == 1:
            # Publish only one line
            x1, y1, x2, y2 = filtered_lines[0]
            m_1 = (y2 - y1) / (x2 - x1)
            b = y1 - m_1 * x1
            x_lane_1 = (y_return - b) / m_1

            p1 = Point()
            p1.x = x_lane_1
            p1.y = y_return

            if m_1 > 0:
                msg.detectedLeft = True
                msg.lineLeft = p1
            else:
                msg.detectedRight = True
                msg.lineRight = p1

        self.lane_pub.publish(msg)

        # Changed from keeping track of positions to checking slopes
        """
        if None in self.prev_line_locations:
            idx_none = self.prev_line_locations.index(None)
            if idx_none == 0:
                self.prev_line_locations = [None, filtered_lines[0]]
                msg.poses = [None, Pose(x_lane_1,y_return)]
                return [None, (x_lane_1,y_return)]
            else:
                self.prev_line_locations = [filtered_lines[0],None]
                msg.poses = [Pose(x_lane_1,y_return), None]
                return [(x_lane_1,y_return), None]
        left_lane = self.prev_line_locations[0]
        right_lane = self.prev_line_locations[1]
        dist_left = abs(left_lane[0]-x1)+abs(left_lane[1]-y1)+abs(left_lane[2]-x2)+abs(left_lane[3]-y2)
        dist_right = abs(right_lane[0]-x1)+abs(right_lane[1]-y1)+abs(right_lane[2]-x2)+abs(right_lane[3]-y2)
        if dist_left < dist_right:
            self.prev_line_locations = [filtered_lines[0],None]
            msg.poses = [Pose(x_lane_1,y_return), None]
            return [(x_lane_1,y_return), None]  
        else:
            self.prev_line_locations = [None, filtered_lines[0]]
            msg.poses = [None, Pose(x_lane_1,y_return)]
            return [None, (x_lane_1,y_return)]
        """

        # More visualization
        """
        i = 0
        for line in filtered_lines:
            x1,y1,x2,y2 = line
            
            cv.line(img, (x1,y1),(x2,y2),(i,255,255 - i),2)
            i += 100
        cv.imshow('Detected Lines', img)
        cv.waitKey(0)
        cv.destroyAllWindows()
        """

    # IDEAS
    """
    # Rrturn line locations after first call back, use these to see where we at
    # Make the line, return it, next time if theres only one, we peep if that one is close by the previous
    # I think homography needs to do the mean of the lines, the pixel mean is probably not accurate for shit
    """


if __name__ == "__main__":
    rospy.init_node("lane_detector")
    ld = LaneDetector()
    rospy.spin()
