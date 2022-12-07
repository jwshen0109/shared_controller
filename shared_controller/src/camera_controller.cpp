#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped init_pose;
geometry_msgs::PoseStamped target_pose;

void poseCallback(const geometry_msgs::PoseStampedConstPtr &last_pose)
{
    target_pose.pose = last_pose->pose;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "camera_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    // subscribe from your topic
    ros::Subscriber pose_sub = nh.subscribe("/Posestamped", 1, &poseCallback);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/left/cartesian_motion_controller/target_frame", 1);
    // ros::Publisher test = nh.advertise<geometry_msgs::PoseStamped>("/test", 1, &poseCallback);

    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = "base_link";
    init_pose.header.stamp = ros::Time::now();
    init_pose.header.frame_id = "base_link";
    init_pose.pose.position.x = 0.183;
    init_pose.pose.position.y = 0.079;
    init_pose.pose.position.z = 0.727;
    init_pose.pose.orientation.x = 0.865;
    init_pose.pose.orientation.y = -0.394;
    init_pose.pose.orientation.z = 0.306;
    init_pose.pose.orientation.w = -0.050;
    target_pose.pose = init_pose.pose;
    // target_pose.pose.position.x = 0.511321365162;
    // target_pose.pose.position.y = 0.0580759870762;
    // target_pose.pose.position.z = 0.459220516652;
    // target_pose.pose.orientation.x = 0.234240052684;
    // target_pose.pose.orientation.y = 0.0574898936857;
    // target_pose.pose.orientation.z = 0.897782382363;
    // target_pose.pose.orientation.w = 0.368528565732;
    int count = 0;

    while (ros::ok())
    {
        if (count < 1000)
        {
            pose_pub.publish(init_pose);
            count++;
        }
        else if (count >= 1000)
        {
            pose_pub.publish(target_pose);
            // count = 0;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}