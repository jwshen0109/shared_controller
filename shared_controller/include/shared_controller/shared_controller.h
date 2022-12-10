#ifndef SHARED_CONTROLLER_H_
#define SHARED_CONTROLLER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <shared_controller/commandConfig.h>

#include <termio.h>
#include <iostream>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

#include "shared_controller/artificial_potential_field.h"

using namespace std;

struct dynamic_reconfigure_params
{
    double ref_force_x = 0.0;
    double scale = 1.0;
};

class SharedController
{

public:
    SharedController();

    void twistCallback(const geometry_msgs::TwistStampedConstPtr &last_msg);

    void buttonCallback(const sensor_msgs::JoyConstPtr &last_state);

    void configCallback(shared_controller::commandConfig &config, uint32_t level);

    void publishFtSensorForce();

    ros::NodeHandle nh;

private:
    ros::Publisher ft_force_pub;
    ros::Publisher reference_force_pub;
    ros::Subscriber sigma_twist_sub;
    ros::Subscriber sigma_button_sub;

    dynamic_reconfigure::Server<shared_controller::commandConfig> server;
    dynamic_reconfigure::Server<shared_controller::commandConfig>::CallbackType f;

    dynamic_reconfigure_params drp;

private:
    // dynamic config
    int first_state = 0;
    int goal = 0;
    bool init = false;

    geometry_msgs::TwistStamped last_twsit;
    vector<double> delta_twist = vector<double>(6, 0.0);
    vector<int> button_states = vector<int>(2, 0.0);
};

class TeleOperation
{
public:
    TeleOperation();

    void callback_left(const geometry_msgs::PoseStampedConstPtr &last_msgs_left);

    void callback_right(const geometry_msgs::PoseStampedConstPtr &last_msgs_right);

    void callback_button_left(const sensor_msgs::JoyConstPtr &last_button_left);

    void callback_button_right(const sensor_msgs::JoyConstPtr &last_button_right);

    void current_pose_callback_left(const geometry_msgs::PoseStampedConstPtr &msgs);

    void current_pose_callback_right(const geometry_msgs::PoseStampedConstPtr &msgs);

    // void configCallback(sigma_client::PathGenerationConfig &config, uint32_t level);

    Eigen::Quaterniond scaleRotation(Eigen::Quaterniond &q_current, double scale);

    std::vector<double> toEulerAngle(Eigen::Quaterniond &q);

public:
    Point *p_current;
    Cylinder *target_cylinder;
    VirtualFixture vf;

private:
    ros::NodeHandle nh;
    ros::Publisher target_pub_left;
    ros::Publisher target_pub_right;
    ros::Publisher path_pub_left;
    ros::Publisher path_pub_right;

    ros::ServiceClient head_client;

    // poses & buttons subscriber
    ros::Subscriber sub_sigma_left;
    ros::Subscriber sub_sigma_right;
    ros::Subscriber sub_sigma_button_left;
    ros::Subscriber sub_sigma_button_right;
    ros::Subscriber sub_current_pose_left;
    ros::Subscriber sub_current_pose_right;

    geometry_msgs::PoseStamped last_pose_left;
    geometry_msgs::PoseStamped last_sigma_left;
    geometry_msgs::PoseStamped last_pose_right;
    geometry_msgs::PoseStamped last_sigma_right;
    geometry_msgs::PoseStamped current_pose_left;
    geometry_msgs::PoseStamped current_pose_right;

    // 0,1,2 for left ur; 3,4,5 for right ur
    vector<double> delta_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // left ur quaternions
    Eigen::Quaterniond delta_q_left;
    Eigen::Quaterniond q_cur_left;
    Eigen::Quaterniond q_last_left;
    Eigen::Quaterniond ur_q_cur_left;
    Eigen::Quaterniond ur_q_target_left;
    Eigen::Quaterniond ur_q_target_left_scale;

    // right ur quaternions
    Eigen::Quaterniond delta_q_right;
    Eigen::Quaterniond q_cur_right;
    Eigen::Quaterniond q_last_right;
    Eigen::Quaterniond ur_q_cur_right;
    Eigen::Quaterniond ur_q_target_right;

    // left sigma quaternions
    Eigen::Quaterniond q_sigma_left;
    Eigen::Quaterniond q_target_left;
    Eigen::Quaterniond q_transform_left;

    // right sigma quaternions
    Eigen::Quaterniond q_sigma_right;
    Eigen::Quaterniond q_target_right;
    Eigen::Quaterniond q_transform_right;

    // button flags
    int button_left = 0;
    int button_right = 0;

    // init state
    int first_flag_left = 0;
    int first_flag_right = 0;

    // dynamic_reconfigure config
    // dynamic_reconfigure::Server<sigma_client::PathGenerationConfig> server;
    // dynamic_reconfigure::Server<sigma_client::PathGenerationConfig>::CallbackType f;
    // int path_config = 0;
};

#endif //