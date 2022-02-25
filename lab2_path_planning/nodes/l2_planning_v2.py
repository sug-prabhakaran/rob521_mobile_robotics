#!/usr/bin/env python3
#Standard Libraries
from importlib.resources import path
from pathlib import Path
from xxlimited import foo
import numpy as np
import yaml
import pygame
import time
import pygame_utils
import matplotlib.image as mpimg
from skimage.draw import circle
from scipy.linalg import block_diag
from math import sin, cos, atan2

#Map Handling Functions
def load_map(filename):
    im = mpimg.imread("../maps/" + filename)
    im_np = np.array(im)  #Whitespace is true, black is false
    #im_np = np.logical_not(im_np)    
    return im_np

def load_map_yaml(filename):
    with open("../maps/" + filename, "r") as stream:
            map_settings_dict = yaml.safe_load(stream)
    return map_settings_dict

#Node for building a graph
class Node:
    def __init__(self, point, parent_id, cost):
        self.point = point # A 3 by 1 vector [x, y, theta]
        self.parent_id = parent_id # The parent node id that leads to this node (There should only every be one parent in RRT)
        self.cost = cost # The cost to come to this node
        self.children_ids = [] # The children node ids of this node
        return

#Path Planner 
class PathPlanner:
    #A path planner capable of performing RRT and RRT*
    def __init__(self, map_filename, map_setings_filename, goal_point, stopping_dist):
        #Get map information
        self.occupancy_map = load_map(map_filename)
        self.map_shape = self.occupancy_map.shape
        self.map_settings_dict = load_map_yaml(map_setings_filename)

        #Get the metric bounds of the map
        self.bounds = np.zeros([2,2]) #m
        self.bounds[0, 0] = self.map_settings_dict["origin"][0]
        self.bounds[1, 0] = self.map_settings_dict["origin"][1]
        self.bounds[0, 1] = self.map_settings_dict["origin"][0] + self.map_shape[0] * self.map_settings_dict["resolution"]
        self.bounds[1, 1] = self.map_settings_dict["origin"][1] + self.map_shape[1] * self.map_settings_dict["resolution"]

        #Robot information
        self.robot_radius = 0.22 #m
        self.vel_max = 0.26 #m/s (Feel free to change!)
        self.rot_vel_max = 0.2 #rad/s (Feel free to change!)

        #Goal Parameters
        self.goal_point = goal_point #m
        self.stopping_dist = stopping_dist #m

        #Trajectory Simulation Parameters
        self.timestep = 1.0 #s
        self.num_substeps = 10

        #Planning storage
        self.nodes = [Node(np.zeros((3,1)), -1, 0)]

        #RRT* Specific Parameters
        self.lebesgue_free = np.sum(self.occupancy_map) * self.map_settings_dict["resolution"] **2
        self.zeta_d = np.pi
        self.gamma_RRT_star = 2 * (1 + 1/2) ** (1/2) * (self.lebesgue_free / self.zeta_d) ** (1/2)
        self.gamma_RRT = self.gamma_RRT_star + .1
        self.epsilon = 2.5

        # controller parameters
        self.kP = 1.0
        self.kD = 0.1
        self.prev_err_head = 0.0
        
        #Pygame window for visualization
        self.window = pygame_utils.PygameWindow(
            "Path Planner", (1000, 1000), self.occupancy_map.shape, self.map_settings_dict, self.goal_point, self.stopping_dist)
        return

    #Functions required for RRT
    def sample_map_space(self):
        #Return an [x,y] coordinate to drive the robot towards
        print("TO DO: Sample point to drive towards")
        return np.zeros((2, 1))
    
    def check_if_duplicate(self, point):
        #Check if point is a duplicate of an already existing node
        print("TO DO: Check that nodes are not duplicates")
        return False
    
    def closest_node(self, point):
        #Returns the index of the closest node
        print("TO DO: Implement a method to get the closest node to a sampled point")
        return 0
    
    def simulate_trajectory(self, node_i, point_s):
        print("\n-----FUNCTION STARTS------\n")
        # Simulates the non-holonomic motion of the robot.
        # This function drives the robot from node_i towards point_s. This function does have many solutions!
        # node_i is a 3 by 1 vector [x;y;theta] this can be used to construct the SE(2) matrix T_{VI} in course notation
        # point_s is the sampled point vector [x; y]
        # inputs:  node_i     (3x1 array) - current robot wrt map frame {I}
        #          point_s    (2x1 array) - goal point in map frame {I}
        # outputs: robot_traj (3xN array) - series of robot poses in map frame {I}

        x, y, theta = node_i[0], node_i[1], node_i[2]         # pos., orient. of robot wrt inertial frame {I}
        print("x, y, theta:", x, y, theta)

        # node_i and point_s are expressed in inertial frame (ie. wrt frame {I}) so need a way to convert to {V}
        C_VI = np.array([[cos(theta), sin(theta)],[-sin(theta), cos(theta)]])
        r_IV_V = -C_VI@np.array((x, y))                             # position vector from {V} to {I} expr. in {V}
        T_VI = np.vstack((np.hstack((C_VI, r_IV_V)), [0, 0, 1]))    # Transformation Matrix from {I} to {V}

        # Simulation Algorithm
        iter = 0

        # 1. Initialize robot_traj
        vel, rot_vel = self.robot_controller(node_i, point_s, self.vel_max, self.kP, self.kD, self.timestep) # initial velocities
        print("initial vel, rot_vel", vel, rot_vel)
        robot_traj = self.trajectory_rollout(vel, rot_vel, theta,
                                             self.timestep, self.num_substeps) + node_i # initial trajectory expressed in {V} frame
        print("\ninitial robot_traj:\n", robot_traj)
        
        cur_node = robot_traj[:, -1].reshape(3,1)
        dist_to_goal = np.linalg.norm(point_s - cur_node[:2])
        print("dist_to_goal:", dist_to_goal)

        while dist_to_goal > 0.3:
            #1. calculate initial vel, rot_vel
            print("cur_node:\n", cur_node, "\npoint_s\n", point_s)
            print("\ncur theta:", cur_node[2])
            vel, rot_vel = self.robot_controller(cur_node, point_s, self.vel_max, self.kP, self.kD, self.timestep)
            print("\niter:", iter, "- vel, rot_vel", vel, rot_vel)

            #2. simulate trajectory for another  timestep and add to existing trajectory
            step_traj = self.trajectory_rollout(vel, rot_vel, cur_node[2], 
                                                self.timestep, self.num_substeps) + cur_node
            print("\nstep_traj:\n", step_traj)
            robot_traj = np.hstack((robot_traj, step_traj))
            #print("\nrobot_traj:\n", robot_traj)

            #3. update current node and dist
            cur_node = robot_traj[:, -1].reshape(3,1)
            dist_to_goal = np.linalg.norm(point_s - cur_node[:2])
            print("outside dist:", dist_to_goal)
            print("\nupdated cur_node:\n", cur_node)
            
            iter += 1
   
        return robot_traj
    
    def robot_controller(self, node_i, point_s, max_vel, kP, kD, delta_t):
        # This controller determines the velocities that will nominally move the robot from node i to node s
        # Max velocities should be enforced
        # inputs: node_i (3x1 array)  - current point of robot wrt robot frame {I}
        #         point_s (2x1 array) - goal point of robot wrt robot frame {I}
        # outputs: vel (float)        - robot velocity wrt inertial frame {I}
        #         rot_vel (float)     - robot angular velocity wrt frame {I}

        # OPTION 1: PID CONTROL

        # calculate head error: angle b/w desired (theta_d) and actual headings (theta)
        theta_d = np.arctan2((point_s[1]-node_i[1]),(point_s[0]-node_i[0])) # desired heading {I} frame
        theta = node_i[2]                                                   # actual heading {I} frame
        err_head = theta_d - theta                                          # heading error
        err_head = np.around(atan2(sin(err_head), cos(err_head)),3)         # normalized heading error

        rot_vel = np.round(kP*(err_head) + kD*(err_head-self.prev_err_head)/(delta_t), 2)
        print("rot_vel:", rot_vel)

        if rot_vel > max_vel:
            rot_vel = max_vel
        if rot_vel < -max_vel:
            rot_vel = -max_vel

        vel = np.around(max_vel/(6*abs(rot_vel)+1),2)
        print("vel:", vel)
        # update controller error values
        self.prev_err_head = err_head                                       # previous error term for kD
        # self.cumul_error_head += error_heading*self.timestep              # cumulative error for kI

        return vel, rot_vel
    
    def trajectory_rollout(self, vel, rot_vel, theta_i, timestep, num_substeps):
        # Given your chosen velocities determine the trajectory of the robot for your given timestep
        # The returned trajectory should be a series of points in {I} frame to check for collisions
        # inputs: vel (float)                  - robot velocity wrt inertial frame {I}
        #         rot_vel (float)              - robot angular velocity wrt frame {I}
        # output: self.trajectory (3x10 array) - robot pose for each time-substep expressed in {I} frame

        trajectory = np.array([[],[],[]])                          # initialize array
        t = np.linspace(0, timestep, num_substeps+1)
    
        if rot_vel == 0:
            x_I = [np.around((vel*t*np.cos(theta_i)),2)]
            y_I = [np.around((vel*t*np.sin(theta_i)),2)]
            theta_I = [np.zeros(num_substeps+1)]
        else:
            x_I = [np.around((vel/rot_vel)*(np.sin(rot_vel*t + theta_i)-np.sin(theta_i)), 4)]       # position in {V} frame
            y_I = [np.around((vel/rot_vel)*(np.cos(theta_i)-np.cos(rot_vel*t + theta_i)), 4)]
            #print("\nx_components: vel/rot_vel", vel/rot_vel, "np.sin(rot_vel*t)", np.sin(rot_vel*t), "-np.sin(theta):", -np.sin(theta_i))
            #print("y_components: vel/rot_vel", vel/rot_vel, "np.cos(theta)", np.cos(theta_i), "-np.sin(theta):", -np.cos(rot_vel*t))

            theta_I = [np.around(rot_vel*t, 4)]                          # orientation in {V}

        trajectory = np.vstack((x_I, y_I, theta_I))
        return trajectory
    
    def point_to_cell(self, point):
        # Convert a series of [x,y] points in the map to the indices for the corresponding cell in the occupancy map
        # point is a 2 by N matrix of points of interest
        # input: point (2xN array)        - points of interest expressed in origin (map) reference frame {I}
        # output: map_indices (2xN array) - points converted to indices wrt top left
        
        # convert from map reference frame {I} to bottom-left ref frame {B}
        # position vector: r_B = r_I + r_BI = r_I - r_IB (offset vector from yaml file)
        x_B = point[0] - self.map_settings_dict["origin"][0] 
        y_B = point[1] - self.map_settings_dict["origin"][1]

        # need to convert to index by dividing by resolution (*1/0.05 = *20)
        height = self.map_shape[1]*self.map_settings_dict["resolution"]          # map height in meters
        x_idx = (x_B/self.map_settings_dict["resolution"]).astype(int)
        y_idx = ((height-y_B)/self.map_settings_dict["resolution"]).astype(int)  # y_B is wrt bottom left, while y_idx is wrt top left
        map_indices = np.vstack((x_idx,y_idx))

        return map_indices

    def points_to_robot_circle(self, points):
        # Convert a series of [x,y] points to robot map footprints for collision detection
        # Hint: The disk function is included to help you with this function
        points_idx = self.point_to_cell(points)         # convert to occupancy grid indexes (pixels)
        
        pixel_radius = self.robot_radius*20         # robot radius in pixels
        footprint = [[],[]]

        for j in range(len(points_idx[0])):
            rr, cc = circle(points_idx[0,j], points_idx[1,j], pixel_radius, shape=(1600,1600))
            footprint = np.hstack((footprint,np.vstack((rr,cc))))
        
        return footprint
    #Note: If you have correctly completed all previous functions, then you should be able to create a working RRT function

    #RRT* specific functions
    def ball_radius(self):
        #Close neighbor distance
        card_V = len(self.nodes)
        return min(self.gamma_RRT * (np.log(card_V) / card_V ) ** (1.0/2.0), self.epsilon)
    
    def connect_node_to_point(self, node_i, point_f):
        #Given two nodes find the non-holonomic path that connects them
        #Settings
        #node is a 3 by 1 node
        #point is a 2 by 1 point
        print("TO DO: Implement a way to connect two already existing nodes (for rewiring).")
        return np.zeros((3, self.num_substeps))
    
    def cost_to_come(self, trajectory_o):
        #The cost to get to a node from lavalle 
        print("TO DO: Implement a cost to come metric")
        return 0
    
    def update_children(self, node_id):
        #Given a node_id with a changed cost, update all connected nodes with the new cost
        print("TO DO: Update the costs of connected nodes after rewiring.")
        return

    #Planner Functions
    def rrt_planning(self):
        #This function performs RRT on the given map and robot
        #You do not need to demonstrate this function to the TAs, but it is left in for you to check your work
        for i in range(1): #Most likely need more iterations than this to complete the map!
            #Sample map space
            point = self.sample_map_space()

            #Get the closest point
            closest_node_id = self.closest_node(point)

            #Simulate driving the robot towards the closest point
            trajectory_o = self.simulate_trajectory(self.nodes[closest_node_id].point, point)

            #Check for collisions
            print("TO DO: Check for collisions and add safe points to list of nodes.")
            
            #Check if goal has been reached
            print("TO DO: Check if at goal point.")
        return self.nodes
    
    def rrt_star_planning(self):
        #This function performs RRT* for the given map and robot        
        for i in range(1): #Most likely need more iterations than this to complete the map!
            #Sample
            point = self.sample_map_space()

            #Closest Node
            closest_node_id = self.closest_node(point)

            #Simulate trajectory
            trajectory_o = self.simulate_trajectory(self.nodes[closest_node_id].point, point)

            #Check for Collision
            print("TO DO: Check for collision.")

            #Last node rewire
            print("TO DO: Last node rewiring")

            #Close node rewire
            print("TO DO: Near point rewiring")

            #Check for early end0.4
            path.append(self.nodes[current_node_id].point)
            current_node_id = self.nodes[current_node_id].parent_id
        path.reverse()
        return path

def main():
    #Set map information
    map_filename = "willowgarageworld_05res.png"
    map_setings_filename = "willowgarageworld_05res.yaml"

    im_np = load_map(map_filename)
    #print('size:', np.shape(im_np))
    #print(im_np)

    #robot information
    goal_point = np.array([[20], [-5]]) #m
    stopping_dist = 0.5 #m

    #RRT precursor
    path_planner = PathPlanner(map_filename, map_setings_filename, goal_point, stopping_dist)

    print(path_planner.bounds)

    #Task 1A test: point_to_cell function 
    # print("Task 1A test: point_to_cell function")
    # point = np.array([[10, 5, -10, 0, 10, -5],
    #                   [10, 5, -10, 0, -5, 10]])
    # print(path_planner.point_to_cell(point))
    # for i in point.T:
    #     path_planner.window.add_point(i, radius=5, color=(0, 0, 255))

    
    # #Task 1B test: points_to_robot_circle function
    # print("/n Task 1B test: points_to_robot_circle function")
    # footprint = path_planner.points_to_robot_circle(point)
    # print(footprint)

    #Task 2A test: trajectory_rollout function
    # print("\nTask 2A test: trajectory_rollout function")
    # traj_rollout = path_planner.trajectory_rollout(8,0.4)
    # print("trajectory_rollout:", traj_rollout)
    # for i, val in enumerate(traj_rollout.T):
    #     if i%2==0:
    #         continue
    #     path_planner.window.add_se2_pose(val, length=8, color=(0,0,255))

    #Task 2B test: robot_controller function
    # print("\nTask 2B test: robot_controller function")
    # node_i = np.array([[0, 0, -1.57]]).T
    # point_s = goal_point  
    # vel, rot_vel = path_planner.robot_controller(node_i, point_s)
    # print("final vel, rot_vel:", vel, rot_vel)
    # best_traj = path_planner.trajectory_rollout(vel, rot_vel)
    # print('best_traj:', best_traj)
    # for i, val in enumerate(best_traj.T):
    #     if i%2==0:
    #         continue
    #     path_planner.window.add_se2_pose(val, length=8, color=(255,0,0))
    
    #Task 2C test: simulate_trajectory function
    print("\nTask 2C test: simulate_trajectory function")
    node_i = np.array([[0, 0, 2.5]]).T
    point_s = goal_point  
    final_trajectory = path_planner.simulate_trajectory(node_i, point_s)
    print("Final Trajectory:", final_trajectory)
    print("shape", np.shape(final_trajectory))

    for i, val in enumerate(final_trajectory.T):
        if i%10 != 0:
            continue
        path_planner.window.add_se2_pose(val, length=8, color=(0, 0, 255))
    
    path_planner.window.add_se2_pose(node_i.flatten(), length=12, color=(255, 0, 0))

    # Ensures that pygame window does not close unless keyboard exit (CTRL+C)
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    #nodes = path_planner.rrt_star_planning()
    #node_path_metric = np.hstack(path_planner.recover_path())

    #Leftover test functions
    #np.save("shortest_path.npy", node_path_metric)

if __name__ == '__main__':
    main()