#!/usr/bin/env python3
from __future__ import division, print_function
import os

import numpy as np
from scipy.linalg import block_diag
from scipy.spatial.distance import cityblock
import rospy
import tf2_ros
from l2_planning_v2 import PathPlanner                              # import functions from PathPlanner() Class
from math import sin, cos, atan2

# msgs
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from visualization_msgs.msg import Marker

# ros and se2 conversion utils
import utils


TRANS_GOAL_TOL = .1  # m, tolerance to consider a goal complete
ROT_GOAL_TOL = .3  # rad, tolerance to consider a goal complete
TRANS_VEL_OPTS = [0, 0.025, 0.13, 0.26]  # m/s, max of real robot is .26
ROT_VEL_OPTS = np.linspace(-1.82, 1.82, 11)  # rad/s, max of real robot is 1.82
CONTROL_RATE = 5  # Hz, how frequently control signals are sent
CONTROL_HORIZON = 5  # seconds. if this is set too high and INTEGRATION_DT is too low, code will take a long time to run!
INTEGRATION_DT = .025  # s, delta t to propagate trajectories forward by
COLLISION_RADIUS = 0.225  # m, radius from base_link to use for collisions, min of 0.2077 based on dimensions of .281 x .306
ROT_DIST_MULT = .1  # multiplier to change effect of rotational distance in choosing correct control
OBS_DIST_MULT = .1  # multiplier to change the effect of low distance to obstacles on a path
MIN_TRANS_DIST_TO_USE_ROT = TRANS_GOAL_TOL  # m, robot has to be within this distance to use rot distance in cost
PATH_NAME = 'path.npy'  # saved path from l2_planning.py, should be in the same directory as this file

# here are some hardcoded paths to use if you want to develop l2_planning and this file in parallel
TEMP_HARDCODE_PATH = [[2, 0, 0], [2.75, -1, -np.pi/2], [2.75, -4, -np.pi/2], [2, -4.4, np.pi]]  # almost collision-free
# TEMP_HARDCODE_PATH = [[2, -.5, 0], [2.4, -1, -np.pi/2], [2.45, -3.5, -np.pi/2], [1.5, -4.4, np.pi]]  # some possible collisions


class PathFollower():
    def __init__(self):
        # time full path
        self.path_follow_start_time = rospy.Time.now()
        print("Start Time:", self.path_follow_start_time)

        # use tf2 buffer to access transforms between existing frames in tf tree
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1.0)  # time to get buffer running

        # constant transforms
        print("Current Time (1):", rospy.Time.now())
        self.map_odom_tf = self.tf_buffer.lookup_transform('map', 'odom', rospy.Time(0), rospy.Duration(2.0)).transform

        # subscribers and publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.global_path_pub = rospy.Publisher('~global_path', Path, queue_size=1, latch=True)
        self.local_path_pub = rospy.Publisher('~local_path', Path, queue_size=1)
        self.collision_marker_pub = rospy.Publisher('~collision_marker', Marker, queue_size=1)

        # map
        map = rospy.wait_for_message('/map', OccupancyGrid)
        self.map_np = np.array(map.data).reshape(map.info.height, map.info.width)
        self.map_resolution = round(map.info.resolution, 5)
        self.map_origin = -utils.se2_pose_from_pose(map.info.origin)  # negative because of weird way origin is stored
        self.map_nonzero_idxes = np.argwhere(self.map_np)

        # collisions
        self.collision_radius_pix = COLLISION_RADIUS / self.map_resolution
        self.collision_marker = Marker()
        self.collision_marker.header.frame_id = '/map'
        self.collision_marker.ns = '/collision_radius'
        self.collision_marker.id = 0
        self.collision_marker.type = Marker.CYLINDER
        self.collision_marker.action = Marker.ADD
        self.collision_marker.scale.x = COLLISION_RADIUS * 2
        self.collision_marker.scale.y = COLLISION_RADIUS * 2
        self.collision_marker.scale.z = 1.0
        self.collision_marker.color.g = 1.0
        self.collision_marker.color.a = 0.5

        # transforms
        self.map_baselink_tf = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
        self.pose_in_map_np = np.zeros(3)
        self.pos_in_map_pix = np.zeros(2)
        self.update_pose()

        # path variables
        cur_dir = os.path.dirname(os.path.realpath(__file__))

        # to use the temp hardcoded paths above, switch the comment on the following two lines
        # self.path_tuples = np.load(os.path.join(cur_dir, 'path.npy')).T
        self.path_tuples = np.array(TEMP_HARDCODE_PATH)

        #Path is series of geometry_msgs.msg.PoseStamped()
        self.path = utils.se2_pose_list_to_path(self.path_tuples, 'map')
        self.global_path_pub.publish(self.path)

        # goal
        self.cur_goal = np.array(self.path_tuples[0])
        self.cur_path_index = 0

        # trajectory rollout tools
        self.kP = 1
        self.kD = 0
        self.prev_err_head = 0
        # self.all_opts is a Nx2 array with all N possible combinations of the t and v vels, scaled by integration dt
        self.all_opts = np.around(np.array(np.meshgrid(TRANS_VEL_OPTS, ROT_VEL_OPTS)).T.reshape(-1, 2),3)
        

        # if there is a [0, 0] option, remove it
        all_zeros_index = (np.abs(self.all_opts) < [0.001, 0.001]).all(axis=1).nonzero()[0]
        if all_zeros_index.size > 0:
            self.all_opts = np.delete(self.all_opts, all_zeros_index, axis=0)
        self.all_opts_scaled = self.all_opts * INTEGRATION_DT
        
        self.num_opts = self.all_opts_scaled.shape[0]
        self.horizon_timesteps = int(np.ceil(CONTROL_HORIZON / INTEGRATION_DT))
        self.iteration = 1
        #print("Horizon Timesteps:", self.horizon_timesteps)

        self.rate = rospy.Rate(CONTROL_RATE)

        rospy.on_shutdown(self.stop_robot_on_shutdown)
        self.follow_path()

    def follow_path(self):
        while not rospy.is_shutdown():
            # timing for debugging...loop time should be less than 1/CONTROL_RATE
            print(self.iteration, ": Current Time:", rospy.Time.now())      # time stored as Time/Duration, not float

            tic = rospy.Time.now()

            self.update_pose()
            self.check_and_update_goal()

            # PART1: start trajectory rollout algorithm
            
            # initialize 3D vector local_path to store trajectory data (200 time steps) X 43 options X pose: [x, y, theta] 
            local_paths = np.zeros([self.horizon_timesteps + 1, self.num_opts, 3])
            local_paths[0] = np.atleast_2d(self.pose_in_map_np).repeat(self.num_opts, axis=0)

            # iterate through 43 options and store projected trajectory for each in local path
            for i, option in enumerate(self.all_opts):
                traj = PathPlanner.trajectory_rollout(PathPlanner, option[0], option[1], 
                                                      local_paths[0,i,2], CONTROL_HORIZON, 
                                                      self.horizon_timesteps).T
                local_paths[:, i] = np.around(traj + local_paths[0,i].reshape(1,3),4)            

            # PART2: check all trajectory points for collisions
            
            # first find the closest collision point in the map to each local path point
            local_paths_pixels = (self.map_origin[:2] + local_paths[:, :, :2]) / self.map_resolution
            valid_opts = range(self.num_opts)
            local_paths_lowest_collision_dist = np.ones(self.num_opts) * 50

            print("TO DO: Check the points in local_path_pixels for collisions")
            '''COLLISION CHECKING PSEUDOCODE
            1. Use self.map_non_zero_idxes, filters only pts < 20 pixels away from our current location
                    (a) uses np.linalg.norm( difference between map_non_zero_idx and current loc in pixels)
                    (b) note*** the indexes for pos_in_np_pixels are reversed.
            2. Iterate through 43 options:
                Check for collisons for all points on path
                If collision, remove option from options   
            3. For each path, calculate distance to closest obstacle.
            '''

            for opt in range(local_paths_pixels.shape[1]):
                for timestep in range(local_paths_pixels.shape[0]):
                    pass

            # remove trajectories that were deemed to have collisions
            print("TO DO: Remove trajectories with collisions!")

            # PART3: calculate final cost and choose best option
            print("\n----CONTROL CALCULATIONS----\n")

            # initialize parameters
            self.delta_time = (rospy.Time.now() - tic).to_sec()         # time for one iteration of while loop
            print("delta_t:", self.delta_time)
            final_cost = np.zeros(self.num_opts)                        # initialize final_cost array [1 x 43]
    
            # iterate through 43 options and calculate heading error, velocity error for each
            for i, option in enumerate(self.all_opts):
                PathPlanner.prev_err_head = 0                           # required for robot_controller function
                # heading error between angle from point to goal and actual pose
                desired_heading = np.around(np.arctan2((self.cur_goal[1]-self.pose_in_map_np[1]),
                                                       (self.cur_goal[0]-self.pose_in_map_np[0])), 4)
                actual_heading = np.around(self.pose_in_map_np[2], 4)
                heading_error = np.around(self.normalize_angle(desired_heading-actual_heading), 4) # normalize

                # calculate what ideal vel, rot_vel would be according to simulation
                vel, rot_vel = PathPlanner.robot_controller(PathPlanner, self.pose_in_map_np, self.cur_goal,
                                                            TRANS_VEL_OPTS[-1], self.kP, self.kD, self.delta_time)
                
                velocity_error = np.around(abs(vel-option[0]) + 10*abs(rot_vel-option[1]), 4)

                print("\n--ERROR CALCS FOR DEBUGGING--")
                print('self.cur_goal:', self.cur_goal, 'self.pose_in_map:', self.pose_in_map_np) 
                print("heading error calc - desired:", desired_heading, "actual:", actual_heading, "error:", heading_error)
                print("vel error calc - desired:", vel, rot_vel, "actual:", option, "error:", velocity_error)
                final_cost[i] = velocity_error + heading_error

            print("final_cost:", final_cost)

            if final_cost.size == 0:  # hardcoded recovery if all options have collision
                control = [-.1, 0]
            else:
                best_opt = valid_opts[final_cost.argmin()]
                control = self.all_opts[best_opt]
                print("best_opt:", final_cost.argmin(), best_opt)
                self.local_path_pub.publish(utils.se2_pose_list_to_path(local_paths[:, best_opt], 'map'))
            print("CONTROL INPUT:", control)

            # send command to robot
            self.cmd_pub.publish(utils.unicyle_vel_to_twist(control))

            # uncomment out for debugging if necessary
            #print("Selected control: {control}, Loop time: {time}, Max time: {max_time}".format(
            #    control=control, time=(rospy.Time.now() - tic).to_sec(), max_time=1/CONTROL_RATE))

            self.rate.sleep()
            self.iteration +=1

            # if self.iteration > 3:
            #     break

    def update_pose(self):
        # Update numpy poses with current pose using the tf_buffer
        self.map_baselink_tf = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0)).transform
        self.pose_in_map_np[:] = [self.map_baselink_tf.translation.x, self.map_baselink_tf.translation.y,
                                  utils.euler_from_ros_quat(self.map_baselink_tf.rotation)[2]]
        print("Current Robot POSE:", self.pose_in_map_np)
        self.pos_in_map_pix = (self.map_origin[:2] + self.pose_in_map_np[:2]) / self.map_resolution
        self.collision_marker.header.stamp = rospy.Time.now()
        self.collision_marker.pose = utils.pose_from_se2_pose(self.pose_in_map_np)
        self.collision_marker_pub.publish(self.collision_marker)

    def check_and_update_goal(self):
        # iterate the goal if necessary
        dist_from_goal = np.linalg.norm(self.pose_in_map_np[:2] - self.cur_goal[:2])
        abs_angle_diff = np.abs(self.pose_in_map_np[2] - self.cur_goal[2])
        rot_dist_from_goal = min(np.pi * 2 - abs_angle_diff, abs_angle_diff)
        if dist_from_goal < TRANS_GOAL_TOL and rot_dist_from_goal < ROT_GOAL_TOL:
            rospy.loginfo("Goal {goal} at {pose} complete.".format(
                    goal=self.cur_path_index, pose=self.cur_goal))
            if self.cur_path_index == len(self.path_tuples) - 1:
                rospy.loginfo("Full path complete in {time}s! Path Follower node shutting down.".format(
                    time=(rospy.Time.now() - self.path_follow_start_time).to_sec()))
                rospy.signal_shutdown("Full path complete! Path Follower node shutting down.")
            else:
                self.cur_path_index += 1
                self.cur_goal = np.array(self.path_tuples[self.cur_path_index])
        else:
            rospy.logdebug("Goal {goal} at {pose}, trans error: {t_err}, rot error: {r_err}.".format(
                goal=self.cur_path_index, pose=self.cur_goal, t_err=dist_from_goal, r_err=rot_dist_from_goal
            ))

    def stop_robot_on_shutdown(self):
        self.cmd_pub.publish(Twist())
        rospy.loginfo("Published zero vel on shutdown.")

    def normalize_angle(self, theta):
        return atan2(sin(theta), cos(theta))


if __name__ == '__main__':
    try:
        rospy.init_node('path_follower', log_level=rospy.DEBUG)
        pf = PathFollower()
    except rospy.ROSInterruptException:
        pass