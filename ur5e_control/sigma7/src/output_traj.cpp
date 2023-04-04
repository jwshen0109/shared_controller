#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <vector>

#include <fstream>

using namespace std;

ofstream output_lm;
ofstream output_rm;
ofstream output_ls;
ofstream output_rs;

int button_left = 0;
int button_right = 0;
vector<float> last_leftmaster = vector<float>(3, 0.0f);
vector<float> last_rightmaster = vector<float>(3, 0.0f);

void leftmasterCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    if (button_left == 1)
    {
        float x = (pose->pose.position.x - last_leftmaster[0]) + 0.45;
        float y = (pose->pose.position.y - last_leftmaster[1]);
        float z = (pose->pose.position.z - last_leftmaster[2]) + 0.4;
        output_lm << y + 0.45 << "\t";
        output_lm << -(x - 0.45) << "\t";
        output_lm << z << endl;
    }
    else
    {
        last_leftmaster[0] = pose->pose.position.x;
        last_leftmaster[1] = pose->pose.position.y;
        last_leftmaster[2] = pose->pose.position.z;
    }
}

void rightmasterCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    if (button_right == 1)
    {
        float x = (pose->pose.position.x - last_rightmaster[0]) + 0.45;
        float y = (pose->pose.position.y - last_rightmaster[1]);
        float z = (pose->pose.position.z - last_rightmaster[2]) + 0.4;
        output_rm << -(y - 0.45) << "\t";
        output_rm << x - 0.45 << "\t";
        output_rm << z << endl;
    }
    else
    {
        last_rightmaster[0] = pose->pose.position.x;
        last_rightmaster[1] = pose->pose.position.y;
        last_rightmaster[2] = pose->pose.position.z;
    }
}

void leftslaveCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    if (button_left == 1)
    {
        output_ls << pose->pose.position.x << "\t";
        output_ls << pose->pose.position.y << "\t";
        output_ls << pose->pose.position.z << std::endl;
    }
}

void rightslaveCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    if (button_right == 1)
    {
        output_rs << pose->pose.position.x << "\t";
        output_rs << pose->pose.position.y << "\t";
        output_rs << pose->pose.position.z << std::endl;
    }
}

void callback_button_left(const sensor_msgs::JoyConstPtr &last_button_left)
{
    button_left = last_button_left->buttons[0];
}

void callback_button_right(const sensor_msgs::JoyConstPtr &last_button_right)
{
    button_right = last_button_right->buttons[0];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "op_traj");
    ros::NodeHandle nh;

    ros::Subscriber ls_pose_sub = nh.subscribe("/left/cartesian_motion_controller/current_pose", 1, &leftslaveCallback);
    ros::Subscriber rs_pose_sub = nh.subscribe("/right/cartesian_motion_controller/current_pose", 1, &rightslaveCallback);
    ros::Subscriber lm_pose_sub = nh.subscribe("/sigma7/sigma1/pose", 1, &leftmasterCallback);
    ros::Subscriber rm_pose_sub = nh.subscribe("/sigma7/sigma0/pose", 1, &rightmasterCallback);
    ros::Subscriber sub_sigma_button_left = nh.subscribe("/sigma7/sigma1/buttons", 1, &callback_button_left);
    ros::Subscriber sub_sigma_button_right = nh.subscribe("/sigma7/sigma0/buttons", 1, &callback_button_right);

    output_lm.open("/home/ur5e/Code/force_control/data/lm.txt");
    output_rm.open("/home/ur5e/Code/force_control/data/rm.txt");
    output_ls.open("/home/ur5e/Code/force_control/data/ls.txt");
    output_rs.open("/home/ur5e/Code/force_control/data/rs.txt");

    ros::Rate loop_rate(100);
    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}