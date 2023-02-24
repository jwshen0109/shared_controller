#!/usr/bin/env python
# coding:utf-8
import rospy
from sword.srv import *


def client_srv():
    rospy.init_node('greetings_client')
    # 等待有可用的服务 "greetings"
    rospy.wait_for_service("infer_many")
    try:
        # 定义service客户端，service名称为“greetings”，service类型为Greeting
        greetings_client = rospy.ServiceProxy("infer_many", Stick)

        # 向server端发送请求，发送的request内容为name和age,其值分别为"HAN", 20
        # 此处发送的request内容与srv文件中定义的request部分的属性是一致的
        #resp = greetings_client("HAN",20)
        for i in range(1):
            resp = greetings_client.call(
                cur_p=[0, 0, 0],
                cur_r=[0, 0, 0, 1],
                tar_p=[0, 0, 0],
                tar_r=[0, 0, 0, 1],
                length=1.0,
            )
            rospy.loginfo("Message From server:%s" % resp)
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)


def client_srv2():
    rospy.init_node('optitrack_client')
    # 等待有可用的服务 "greetings"
    rospy.wait_for_service("optitrack")
    try:
        # 定义service客户端，service名称为“greetings”，service类型为Greeting
        greetings_client = rospy.ServiceProxy("optitrack", Rigid)

        for i in range(1):
            resp = greetings_client.call(
                idx=0
            )
            rospy.loginfo("Message From server:%s" % resp)
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)


# 如果单独运行此文件，则将上面函数client_srv()作为主函数运行
if __name__ == "__main__":
    client_srv2()
