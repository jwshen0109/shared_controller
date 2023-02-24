#!/usr/bin/env python
# coding=utf-8
import sys
sys.path.append(
    "/home/tric/Downloads/NatNet_SDK_4.0_ubuntu/lib/PythonClient")

import rospy
from sword.srv import *
import sys
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData
import numpy as np


def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len = len(arg_list)
    if arg_list_len > 1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len > 2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len > 3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False

    return args_dict


class OptiTrackClient:
    def __init__(self) -> None:
        optionsDict = {}
        optionsDict["clientAddress"] = "127.0.0.1"
        optionsDict["serverAddress"] = "192.168.1.117"
        optionsDict["use_multicast"] = True

        # This will create a new NatNet client
        optionsDict = my_parse_args(sys.argv, optionsDict)

        streaming_client = NatNetClient()
        streaming_client.set_client_address(optionsDict["clientAddress"])
        streaming_client.set_server_address(optionsDict["serverAddress"])
        streaming_client.set_use_multicast(optionsDict["use_multicast"])
        self.client = streaming_client

    def get_rigidbody(self):
        return self.client.get_rigidbody()

    def get_pose(self, idx):
        ret = self.get_rigidbody()
        # print(ret.get_as_string())
        rigid_body = ret.rigid_body_data.rigid_body_list[idx]
        marker_data = ret.marker_set_data.marker_data_list[idx]
        return np.array(rigid_body.pos), np.array(rigid_body.rot), marker_data.marker_pos_list


if __name__ == "__main__":
    rospy.init_node("optitrack_server")
    streaming_client = OptiTrackClient()

    def handle_function(req: RigidRequest):
        rospy.loginfo('Request from %d', req.idx)
        ret = streaming_client.get_rigidbody()
        print(ret.get_as_string())
        return RigidResponse(pos=ret.rigid_body_list[req.idx].pos, rot=ret.rigid_body_list[req.idx].rot)

    s = rospy.Service("optitrack", Rigid, handle_function)
    rospy.loginfo("Ready to handle the request:")
    # 阻塞程序结束
    rospy.spin()
