#!/usr/bin/env python
from __future__ import division, print_function

import numpy as np
import rospy
import tf2_ros
from skimage.draw import line as ray_trace
import math

# msgs
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

from utils import convert_pose_to_tf, convert_tf_to_pose, euler_from_ros_quat, \
     tf_to_tf_mat, tf_mat_to_tf


ALPHA = 4
BETA = 1
# MAP_DIM = (4, 4)
MAP_DIM = (6, 6)
CELL_SIZE = .01
NUM_PTS_OBSTACLE = 3
SCAN_DOWNSAMPLE = 1

class OccupancyGripMap:
    def __init__(self):
        # use tf2 buffer to access transforms between existing frames in tf tree
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_br = tf2_ros.TransformBroadcaster()

        # subscribers and publishers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

        # attributes
        width = int(MAP_DIM[0] / CELL_SIZE); height = int(MAP_DIM[1] / CELL_SIZE)
        self.log_odds = np.zeros((width, height))
        self.np_map = np.ones((width, height), dtype=np.uint8) * -1  # -1 for unknown
        self.map_msg = OccupancyGrid()
        self.map_msg.info = MapMetaData()
        self.map_msg.info.resolution = CELL_SIZE
        self.map_msg.info.width = width
        self.map_msg.info.height = height

        # transforms
        self.base_link_scan_tf = self.tf_buffer.lookup_transform('base_link', 'base_scan', rospy.Time(0),
                                                            rospy.Duration(2.0))
        odom_tf = self.tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(2.0)).transform

        # set origin to center of map
        rob_to_mid_origin_tf_mat = np.eye(4)
        rob_to_mid_origin_tf_mat[0, 3] = -width / 2 * CELL_SIZE
        rob_to_mid_origin_tf_mat[1, 3] = -height / 2 * CELL_SIZE
        odom_tf_mat = tf_to_tf_mat(odom_tf)
        self.map_msg.info.origin = convert_tf_to_pose(tf_mat_to_tf(odom_tf_mat.dot(rob_to_mid_origin_tf_mat)))

        # map to odom broadcaster
        self.map_odom_timer = rospy.Timer(rospy.Duration(0.1), self.broadcast_map_odom)
        self.map_odom_tf = TransformStamped()
        self.map_odom_tf.header.frame_id = 'map'
        self.map_odom_tf.child_frame_id = 'odom'
        self.map_odom_tf.transform.rotation.w = 1.0

        rospy.spin()

    def broadcast_map_odom(self, e):
        self.tf_br.sendTransform(self.map_odom_tf)

    def scan_cb(self, scan_msg):
        # read new laser data and populate map
        # get current odometry robot pose
        try:
            odom_tf = self.tf_buffer.lookup_transform('odom', 'base_scan', rospy.Time(0)).transform
        except tf2_ros.TransformException:
            rospy.logwarn('Pose from odom lookup failed. Using origin as odom.')
            odom_tf = convert_pose_to_tf(self.map_msg.info.origin)

        # get odom in frame of map
        odom_map_tf = tf_mat_to_tf(
            np.linalg.inv(tf_to_tf_mat(convert_pose_to_tf(self.map_msg.info.origin))).dot(tf_to_tf_mat(odom_tf))
        )
        odom_map = np.zeros(3)
        odom_map[0] = odom_map_tf.translation.x
        odom_map[1] = odom_map_tf.translation.y
        odom_map[2] = euler_from_ros_quat(odom_map_tf.rotation)[2]
        print('Robot position: ', odom_map[0], odom_map[1])
        # loop through all range measurements

        # YOUR CODE HERE!!! Loop through each measurement in scan_msg to get the correct angle and
        # # x_start and y_start to send to your ray_trace_update function.
        for i in range(len(scan_msg.ranges)):
            if scan_msg.ranges[i]<scan_msg.range_max and scan_msg.ranges[i]>scan_msg.range_min:
                cur_angle = odom_map[2] + (scan_msg.angle_min + i*scan_msg.angle_increment)
                cur_range = scan_msg.ranges[i]

                self.ray_trace_update(odom_map[0], odom_map[1], cur_angle, cur_range)

            # break

        # publish the message
        self.map_msg.info.map_load_time = rospy.Time.now()
        self.map_msg.data = self.np_map.flatten()
        self.map_pub.publish(self.map_msg)

    # def ray_trace_update(self, map, log_odds, x_start, y_start, angle, range_mes):
    def ray_trace_update(self, x_start, y_start, angle, range_mes):
        """
        A ray tracing grid update as described in the lab document.

        :param map: The numpy map.
        :param log_odds: The map of log odds values.
        :param x_start: The x starting point in the map coordinate frame (i.e. the x 'pixel' that the robot is in).
        :param y_start: The y starting point in the map coordinate frame (i.e. the y 'pixel' that the robot is in).
        :param angle: The ray angle relative to the x axis of the map.
        :param range_mes: The range of the measurement along the ray.
        :return: The numpy map and the log odds updated along a single ray.
        """

        # YOUR CODE HERE!!! You should modify the log_odds object and the numpy map based on the outputs from
        # ray_trace and the equations from class. Your numpy map must be an array of int8s with 0 to 100 representing
        # probability of occupancy, and -1 representing unknown.
        
        # obstacle position in the inertial reference frame
        obs_x = max(0,x_start + range_mes*math.cos(angle))
        obs_y = max(0,y_start + range_mes*math.sin(angle)) 
        # obs_x = x_start + range_mes*math.cos(angle)
        # obs_y = y_start + range_mes*math.sin(angle)

        tot_points = int(MAP_DIM[0] / CELL_SIZE)-1    # = 399

        if True:
            # mapping the coordinates to from bottom-left axis
            x_start_px = tot_points-int((x_start)*100)
            y_start_px = tot_points-int((y_start)*100)                  
            obs_x_px   = tot_points-int((obs_x)*100)
            obs_y_px   = tot_points-int((obs_y)*100)

            points_x, points_y = ray_trace(x_start_px, y_start_px, obs_x_px, obs_y_px)

            for i in range(len(points_x)-NUM_PTS_OBSTACLE):
                self.log_odds[points_y[i], tot_points-points_x[i]] -=1

            obs_points = -1
            for i in range(NUM_PTS_OBSTACLE):
                self.log_odds[points_y[obs_points], tot_points-points_x[obs_points]] += 1
                obs_points -= 1
            
            self.np_map = self.log_odds_to_probability(self.log_odds)*100

        # return map, log_odds

    def log_odds_to_probability(self, values):
        # print(values)
        return np.exp(values) / (1 + np.exp(values))


if __name__ == '__main__':
    try:
        rospy.init_node('mapping')
        ogm = OccupancyGripMap()
    except rospy.ROSInterruptException:
        pass