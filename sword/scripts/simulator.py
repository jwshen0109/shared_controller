#!/usr/bin/env python
# coding=utf-8
import rospy
from sword.srv import *
import numpy as np
import tf.transformations
from scipy.spatial.transform import Rotation as R


def extract_pose(pose):
    pos = np.array([pose[12], pose[13], pose[14]])
    initial_quaternion = tf.transformations.quaternion_from_matrix(
        np.transpose(np.reshape(pose, (4, 4))))
    rot = initial_quaternion / np.linalg.norm(initial_quaternion)
    return pos, rot


def compose_pose(pos, rot):
    matrix = tf.transformations.compose_matrix(
        None, None, R.from_quat(rot).as_euler("xyz", False), pos, None)
    return matrix.transpose().reshape(-1)


if __name__ == "__main__":
    rospy.init_node('main_client')
    # 等待有可用的服务 "greetings"
    rospy.wait_for_service("backend_infer")
    main_client = rospy.ServiceProxy("backend_infer", Stick)
    poses = np.loadtxt("/home/tric/ros/catkin_ws/pose.txt")
    # the pose of ee
    initial_pose = poses[0]
    target_poses = poses[1:]
    current_pose = initial_pose
    print(extract_pose(initial_pose))
    # exit()
    steps = len(target_poses)
    for target_pose in target_poses[3000:7000:50]:
        cur_p, cur_r = extract_pose(target_pose)
        tar_p, tar_r = extract_pose(target_pose)
        resp = main_client.call(
            cur_p=cur_p,
            cur_r=cur_r,
            tar_p=tar_p,
            tar_r=tar_r
        )
        print(tar_p)
        print(resp.v)
        print('____________________')
        current_pose = compose_pose(resp.v, resp.w)

    # pos_franka = np.array([msg.O_T_EE[12], msg.O_T_EE[13], msg.O_T_EE[14]])
    # # angle, rot, pot = tf.transformations.rotation_from_matrix(np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    # initial_quaternion = \
    #     tf.transformations.quaternion_from_matrix(
    #         np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    # rot_franka = initial_quaternion / \
    #     np.linalg.norm(initial_quaternion)
