# Various convenience functions for conversions, etc.

import numpy as np
import tf_conversions
from geometry_msgs.msg import Transform, Pose, Quaternion


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
    return tf_conversions.transformations.euler_from_quaternion(np_q)

def ros_quat_from_euler(e):
    # get a ROS XYZW quaternion from an SXYZ euler
    np_q = tf_conversions.transformations.quaternion_from_euler(*e)
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
    mat = tf_conversions.transformations.quaternion_matrix(np_q_from_ros_q(tf.rotation))
    mat[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
    return mat

def tf_mat_to_tf(tf_mat):
    # convert 4x4 np se3 matrix to ros transform
    tf = Transform()
    [tf.translation.x, tf.translation.y, tf.translation.z] = tf_mat[:3, 3]
    tf.rotation = ros_q_from_np_q(tf_conversions.transformations.quaternion_from_matrix(tf_mat))
    return tf