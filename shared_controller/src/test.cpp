#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TwistStamped.h>

#include <fstream>

using namespace std;

ofstream outFile;

void velocityCallback(const geometry_msgs::TwistStampedConstPtr &last_msgs)
{
    outFile << last_msgs->twist.linear.x << "\t";
    outFile << last_msgs->twist.linear.y << "\t";
    outFile << last_msgs->twist.linear.z << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_for_velocity");
    ros::NodeHandle nh;
    ros::Subscriber velocity_sub = nh.subscribe("/right/cartesian_motion_controller/current_velocity", 1, velocityCallback);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/right/cartesian_motion_controller/target_frame", 1);
    ros::Rate loop_rate(100);

    outFile.open("/home/ur5e/velocity.txt");
    int count = 0;
    int flag = 0;

    geometry_msgs::PoseStamped target_pose;

    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.4;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.5;
    target_pose.pose.orientation.x = 0.0;
    target_pose.pose.orientation.y = 1.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 0.0;

    while (ros::ok())
    {
        if (count < 1000 && flag == 0)
        {
            target_pose.pose.position.z = 0.3;
            count++;
            if (count == 1000)
            {
                flag = 1;
            }
        }
        else
        {
            target_pose.pose.position.z = 0.5;
            count--;
            if (count == 0)
            {
                flag = 0;
            }
        }
        pose_pub.publish(target_pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}