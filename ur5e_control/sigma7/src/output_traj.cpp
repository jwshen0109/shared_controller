#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <fstream>

using namespace std;

ofstream output_lm;
ofstream output_rm;
ofstream output_ls;
ofstream output_rs;

void leftmasterCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    output_lm << pose->pose.position.x << "\t";
    output_lm << pose->pose.position.y << "\t";
    output_lm << pose->pose.position.z << "\t";
    output_lm << pose->pose.orientation.x << "\t";
    output_lm << pose->pose.orientation.y << "\t";
    output_lm << pose->pose.orientation.z << "\t";
    output_lm << pose->pose.orientation.w << std::endl;
}

void rightmasterCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    output_rm << pose->pose.position.x << "\t";
    output_rm << pose->pose.position.y << "\t";
    output_rm << pose->pose.position.z << "\t";
    output_rm << pose->pose.orientation.x << "\t";
    output_rm << pose->pose.orientation.y << "\t";
    output_rm << pose->pose.orientation.z << "\t";
    output_rm << pose->pose.orientation.w << std::endl;
}

void leftslaveCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    output_ls << pose->pose.position.x << "\t";
    output_ls << pose->pose.position.y << "\t";
    output_ls << pose->pose.position.z << "\t";
    output_ls << pose->pose.orientation.x << "\t";
    output_ls << pose->pose.orientation.y << "\t";
    output_ls << pose->pose.orientation.z << "\t";
    output_ls << pose->pose.orientation.w << std::endl;
}

void rightslaveCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    output_rs << pose->pose.position.x << "\t";
    output_rs << pose->pose.position.y << "\t";
    output_rs << pose->pose.position.z << "\t";
    output_rs << pose->pose.orientation.x << "\t";
    output_rs << pose->pose.orientation.y << "\t";
    output_rs << pose->pose.orientation.z << "\t";
    output_rs << pose->pose.orientation.w << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "op_traj");
    ros::NodeHandle nh;

    ros::Subscriber ls_pose_sub = nh.subscribe("/left/cartesian_force_controller/current_pose", 1, &leftslaveCallback);
    ros::Subscriber rs_pose_sub = nh.subscribe("/right/cartesian_force_controller/current_pose", 1, &rightslaveCallback);
    ros::Subscriber lm_pose_sub = nh.subscribe("/sigma7/sigma0/pose", 1, &leftmasterCallback);
    ros::Subscriber rm_pose_sub = nh.subscribe("/sigma7/sigma1/pose", 1, &rightmasterCallback);

    output_lm.open("/home/ur5e/Code/force_control/data/lm.txt");
    output_rm.open("/home/ur5e/Code/force_control/data/rm.txt");
    output_ls.open("/home/ur5e/Code/force_control/data/ls.txt");
    output_rs.open("/home/ur5e/Code/force_control/data/rs.txt");
}