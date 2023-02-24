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
    # # angle, rot, pot = tf.transformations.rotation_from_matrix(np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    # initial_quaternion = \
    #     tf.transformations.quaternion_from_matrix(
    #         np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    # rot_franka = initial_quaternion / \
    #     np.linalg.norm(initial_quaternion)
    # # get pose of robot in franka

    pos_franka = np.array([0.44600077, 0.12916307, 0.11641214])
    rot_franka = np.array([0.92222274, - 0.38456291, 0.04019363, - 0.00102827])
    print(pos_franka, rot_franka)
    transformer = Transformer(pos_franka, rot_franka)
    ee_pos_world, ee_rot_world = transformer.franka_to_world(
        pos_franka, rot_franka)
    ee_pos_world2, ee_rot_world2 = transformer.get_ee_in_world()
    # fig = plt.figure()
    # ax = plt.axes(projection='3d')
    # ax.scatter3D(ee_pos_world[0], ee_pos_world[1], ee_pos_world[2])
    # plt.xlim([-100, 100])
    # plt.ylim([-100, 100])
    # # plt.zlim([100, 100])
    # plt.show()
    print(ee_pos_world, ee_pos_world2)
    print(ee_rot_world, ee_rot_world2)
    print("---------------")
    points = transformer.world_to_head()
    fpos, frot = transformer.get_flange_in_franka()
    # print(fpos, frot)
    pub = rospy.Publisher("visualization_marker_array",
                          MarkerArray, queue_size=10)
    rate = rospy.Rate(30)
    f = 0.0
    markers = transformer.world_to_franka_points(transformer.markers2)
    # print(markers)
    while not rospy.is_shutdown():
        arr = MarkerArray()
        addMarker(arr, [(pos_franka, None), (fpos, None)] +
                  [(pos, None) for pos in markers] + [(pos / 100, None) for pos in points])
        addLineMarker(arr, (fpos, frot), 0)
        addLineMarker(arr, (pos_franka, rot_franka), 1)
        addLineMarker(arr, (ee_pos_world / 100, ee_rot_world), 2)
        addLineMarker(arr, (ee_pos_world2 / 100, ee_rot_world2), 3)
        pub.publish(arr)
        rate.sleep()

    # setup service
    # send pose
    # use backend ros service
    # return pose

    # rospy.spin()
