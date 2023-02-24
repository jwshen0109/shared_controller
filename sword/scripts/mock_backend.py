#!/usr/bin/env python
# coding=utf-8
import rospy
from sword.srv import *
if __name__ == "__main__":
    rospy.init_node("whole_backend")
    rospy.loginfo("running backend 1111")

    def handle_function(req: StickRequest):
        return StickResponse(v=req.tar_p, w=req.tar_r)

    s = rospy.Service("backend_infer", Stick, handle_function)
    rospy.spin()
