#!/usr/bin/env python
# coding=utf-8

"""
This module is the entry point for the whole backend.
The backend contains three components:
1. OptiTrack Receiver
2. Coordinate Transformer
3. Planning Core

Input: current pose and target pose in **franka coordinate**.
Ouput: actual pose in **franka coordinate**.

Pose -> Transformer(OptiTrack) -> Planning -> Transformer -> Pose
"""
from audioop import add
from os import POSIX_FADV_SEQUENTIAL
import rospy
import numpy as np
import tf.transformations
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from franka_msgs.msg import FrankaState
from scipy.spatial.transform import Rotation as R
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from transformer import Transformer
import rospy
from sword.srv import *

# validation process:
# 1. draw marker in rviz
# 2. since we know the position of flange, marker and ee in world,
#    we can draw them in rviz, to see if that match the real world.
# 3. we can draw head position in rviz.


def addMarker(arr: MarkerArray, poses):
    for i, pose in enumerate(poses):
        m = Marker()
        m.header.frame_id = 'panda_link0'
        m.header.stamp = rospy.Time.now()
        m.ns = f'test{i}'
        m.action = Marker.ADD
        m.id = i
        m.type = Marker.CUBE
        m.color.a = 1.0
        m.color.g = 1.0
        m.scale.x = 0.01
        m.scale.y = 0.01
        m.scale.z = 0.01
        pos = pose[0]
        rot = pose[1]
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = pos[2]
        if rot is not None:
            m.pose.orientation.x = rot[0]
            m.pose.orientation.y = rot[1]
            m.pose.orientation.z = rot[2]
            m.pose.orientation.w = rot[3]
        arr.markers.append(m)


def addLineMarker(arr: MarkerArray, pose, idx):
    m = Marker()
    m.header.frame_id = 'panda_link0'
    m.header.stamp = rospy.Time.now()
    m.ns = f'test_line{idx}'
    m.action = Marker.ADD
    m.id = idx
    m.type = Marker.LINE_STRIP
    m.color.a = 1.0
    m.color.g = 1.0
    m.scale.x = 0.01
    # m.scale.y = 0.01
    # m.scale.z = 0.01
    pos = pose[0]
    rot = pose[1]
    p0 = Point(pos[0], pos[1], pos[2])

    cur_r = R.from_quat(rot)
    direction = cur_r.apply(np.array([0, 0, 1]))
    ext_p = pos + direction * 0.194
    p1 = Point(ext_p[0], ext_p[1], ext_p[2])

    m.points.extend([p0, p1])
    arr.markers.append(m)


if __name__ == "__main__":
    rospy.init_node("whole_backend")
    rospy.loginfo("running backend")

    # msg = rospy.wait_for_message("franka_state_controller/franka_states",
    #                              FrankaState)  # type: FrankaState
    # pos_franka = np.array([msg.O_T_EE[12], msg.O_T_EE[13], msg.O_T_EE[14]])
    # initial_quaternion = \
    #     tf.transformations.quaternion_from_matrix(
    #         np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    # rot_franka = initial_quaternion / \
    #     np.linalg.norm(initial_quaternion)
    # # get pose of robot in franka

    # pos_franka = np.array([0.44600077, 0.12916307, 0.11641214])
    # rot_franka = np.array([0.92222274, - 0.38456291, 0.04019363, - 0.00102827])
    pos_franka = np.array([3.06891e-01, -4.65093e-08, 3.70882e-01])
    rot_franka = np.array(
        [9.23879699e-01, -3.82683030e-01, 7.77763780e-08, 4.78508028e-07])

    transformer = Transformer(pos_franka, rot_franka)

    def handle_function(req: StickRequest):
        # transform franka coordinate to head coordinate
        cur_p_h, cur_r_h = transformer.franka_to_head(req.cur_p, req.cur_r)
        tar_p_h, tar_r_h = transformer.franka_to_head(req.tar_p, req.tar_p)
        # do inference
        # actual_p, actual_r = infer()
        # transform head coordinate back to franka coordinate
        # actual_p_f, actual_r_f = transformer.head_to_franka(tar_p_h, tar_r_h)
        return StickResponse(v=[0, 0, 0], w=[0, 0, 0, 1])

    s = rospy.Service("backend_infer", Stick, handle_function)
    rospy.spin()
