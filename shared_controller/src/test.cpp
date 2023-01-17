#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian_motion_controller/target_frame", 1);
    // 设定循环频率
    ros::Rate loop_rate(100);
    int count = 0;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.header.stamp = ros::Time::now();

    pose.pose.position.x = 0.85;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.5;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 1.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.0;
    while (ros::ok())
    {
        if (count < 500)
        {
            pose.pose.position.y = 0.0;
            // pose.pose.position.z = 0.2;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 1.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 0.0;
            count++;
        }
        else if (count < 1000 && count >= 500)
        {
            pose.pose.position.x = 0.6;
            pose.pose.position.z = 0.45;
            count++;
        }
        else
        {
            pose.pose.position.x = 0.45;
            // pose.pose.position.z = 0.2;
        }
        pose_pub.publish(pose);
        // ROS_INFO("v_msg.linear.x:%lf  v_msg.linear.y:%lf  v_msg.linear.z:%lf", v_msg.linear.x, v_msg.linear.y, v_msg.linear.z);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}