# Various convenience functions for conversions, etc.

import numpy as np
from transforms3d import euler, quaternions
#import tf_conversions
from geometry_msgs.msg import Transform, Pose, Quaternion, PoseStamped, Twist
from nav_msgs.msg import Path

def se2_pose_list_to_path(pose_list, ref_frame):
    # convert a list of poses to a path
    path = Path()
    path.header.frame_id = ref_frame
    for pose in pose_list:
        ros_pose = PoseStamped()
        ros_pose.pose.position.x = pose[0]
        ros_pose.pose.position.y = pose[1]
        ros_pose.pose.orientation = ros_quat_from_euler([0, 0, pose[2]])
        ros_pose.header.frame_id = ref_frame
        path.poses.append(ros_pose)
    return path


def convert_pose_to_tf(pose):
    # convert a ros pose to a ros transform
    transform = Transform()
    transform.translation.x = pose.position.x
    transform.translation.y = pose.position.y
    transform.translation.z = pose.position.z
    transform.rotation = pose.orientation
    return transform


def convert_tf_to_pose(tf):
    # convert a ros transform to a ros pose
    pose = Pose()
    pose.position.x = tf.translation.x
    pose.position.y = tf.translation.y
    pose.position.z = tf.translation.z
    pose.orientation = tf.rotation
    return pose


def euler_from_ros_quat(q):
    # get the SXYZ euler angles from a ros XYZW quaternion
    np_q = np.array([q.x, q.y, q.z, q.w])
    #return tf_conversions.transformations.euler_from_quaternion(np_q)
    return euler.quat2euler(np_q)

def ros_quat_from_euler(e):
    # get a ROS XYZW quaternion from an SXYZ euler
    #np_q = tf_conversions.transformations.quaternion_from_euler(*e)
    np_q = euler.euler2quat(*e)
    return ros_q_from_np_q(np_q) 

def np_q_from_ros_q(q):
    q = np.array([q.x, q.y, q.z, q.w])
    return q


def ros_q_from_np_q(np_q):
    q = Quaternion()
    q.x = np_q[0]; q.y = np_q[1]; q.z = np_q[2]; q.w = np_q[3]
    return q


def tf_to_tf_mat(tf):
    # convert ros transform to 4x4 np se3 matrix
    #mat = tf_conversions.transformations.quaternion_matrix(np_q_from_ros_q(tf.rotation))
    mat = quaternions.quat2mat(np_q_from_ros_q(tf.rotation))
    mat[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
    return mat


def tf_mat_to_tf(tf_mat):
    # convert 4x4 np se3 matrix to ros transform
    tf = Transform()
    [tf.translation.x, tf.translation.y, tf.translation.z] = tf_mat[:3, 3]
    #tf.rotation = ros_q_from_np_q(tf_conversions.transformations.quaternion_from_matrix(tf_mat))
    tf.rotation = ros_q_from_np_q(quaternions.mat2quat(tf_mat[:3,:3]))
    return tf


def tf_to_se2_tf_mat(tf):
    # convert ros transform to 3x3 np se2 matrix
    theta = euler_from_ros_quat(tf.rotation)[2]
    mat = np.array([[np.cos(theta), -np.sin(theta), tf.translation.x],
                    [np.sin(theta), np.cos(theta), tf.translation.y],
                    [0, 0, 1]])
    return mat


def se2_pose_from_pose(pose):
    # convert a ros pose to a (3,) np array in SE2
    return np.array([pose.position.x, pose.position.y, euler_from_ros_quat(pose.orientation)[2]])


def pose_from_se2_pose(np_pose):
    # convert a (3,) np array in SE2 to a ros pose
    p = Pose()
    p.position.x = np_pose[0]
    p.position.y = np_pose[1]
    p.orientation = ros_quat_from_euler(np.array([0, 0, np_pose[2]]))
    return p


def unicyle_vel_to_twist(np_vel):
    # a 2d unicyle model velocity (v, w) to a ros Twist
    v = Twist()
    v.angular.z = np_vel[1]
    v.linear.x = np_vel[0]
    return v