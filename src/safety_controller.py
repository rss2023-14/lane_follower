#!/usr/bin/env python2

import numpy as np
import message_filters

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math


class SafetyController:
    """
    This safety controller is specifically tuned for the Final Challenge Part A!

    Processes desired car driving commands, and stops the car if about to crash.
        - Takes into account car size (HALF_CAR_WIDTH)
        - Predicts the next few seconds (LOOK_AHEAD_TIME)
        - Predicts car's future trajectory based on current speed and turning angle
    """

    LOOK_AHEAD_TIME = rospy.get_param("look_ahead_time", 0.5)  # seconds
    HALF_CAR_WIDTH = rospy.get_param("half_car_width", 0.2)  # meters
    IGNORE_CLOSE_DIST = rospy.get_param("ignore_close_distance", 0.1)  # meters

    def __init__(self):
        self.lidar_sub = message_filters.Subscriber("/scan", LaserScan)
        self.ackermann_sub = message_filters.Subscriber(
            "/vesc/high_level/ackermann_cmd_mux/output", AckermannDriveStamped
        )

        self.num_in_row = 0
        self.num_required = rospy.get_param("detections_required")

        # Initialize safety control publisher
        self.pub = rospy.Publisher(
            "/vesc/low_level/ackermann_cmd_mux/input/safety",
            AckermannDriveStamped,
            queue_size=1,
        )
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.ackermann_sub], 10, 0.05
        )
        ts.registerCallback(self.safety_control)

    def safety_control(self, lidar_msg, ackermann_msg):
        if lidar_msg.angle_increment == 0.0:
            # Early return for uninitialized LaserScan
            return

        # We only care about forward FOV of car (-80 to 80 deg)
        # 1.39626 is 80 degrees in radians
        low_index = self.get_range_index(-1.39626, lidar_msg)
        high_index = self.get_range_index(1.39626, lidar_msg)

        ranges = np.array(lidar_msg.ranges)[low_index:high_index]
        angles = np.arange(
            lidar_msg.angle_min, lidar_msg.angle_max, lidar_msg.angle_increment
        )
        angles = angles[low_index:high_index]

        # Extract turning radius and direction
        if ackermann_msg.drive.steering_angle == 0.0:
            turning_right = 1
            turning_radius = 10000.0  # ~inf
        elif ackermann_msg.drive.steering_angle < 0.0:
            turning_right = 1
            # 0.35 is wheelbase of car
            turning_radius = abs(0.35 / np.tan(ackermann_msg.drive.steering_angle))
        else:
            turning_right = -1
            # 0.35 is wheelbase of car
            turning_radius = abs(0.35 / np.tan(ackermann_msg.drive.steering_angle))

        max_angle = (self.LOOK_AHEAD_TIME * ackermann_msg.drive.speed) / turning_radius

        """
        Take coordinates from the LIDAR's perspective, find trajectory of car.
        """
        for index, range in enumerate(ranges):
            if range < self.IGNORE_CLOSE_DIST:
                # Filter out erroneous LIDAR points
                continue

            # 1.57079633 is 90 degrees in radians
            inner_angle = 1.57079633 - (turning_right * angles[index])

            dist = math.sqrt(
                range**2.0
                + turning_radius**2.0
                - (2.0 * range * turning_radius * np.cos(inner_angle))
            )
            angle = np.arcsin((range / dist) * np.sin(inner_angle))

            if (
                dist > (turning_radius - self.HALF_CAR_WIDTH)
                and dist < (turning_radius + self.HALF_CAR_WIDTH)
                and angle < max_angle
            ):
                self.num_in_row += 1
                if self.num_in_row >= self.num_required:
                    rospy.loginfo("Safety Controller engaged!")

                    msg = AckermannDriveStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "base_link"

                    msg.drive.speed = 0

                    msg.drive.steering_angle_velocity = 0.0
                    msg.drive.acceleration = 0.0
                    msg.drive.jerk = 0.0

                    self.pub.publish(msg)
                    return
                return
        self.num_in_row = 0

    def get_range_index(self, angle, lidar_msg):
        return int((angle - lidar_msg.angle_min) / lidar_msg.angle_increment)


if __name__ == "__main__":
    rospy.init_node("safety_controller")
    safety_controller = SafetyController()
    rospy.spin()
