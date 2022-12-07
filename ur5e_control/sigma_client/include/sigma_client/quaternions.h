#ifndef QUATERNIONS_H_
#define QUATERNIONS_H_

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "nav_msgs/Path.h"
#include <dynamic_reconfigure/server.h>
#include <sigma_client/PathGenerationConfig.h>

#include <iostream>
#include <boost/thread.hpp>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;

class quaTransform
{

public:
    quaTransform();

    void callback_left(const geometry_msgs::PoseStampedConstPtr &last_msgs_left);

    void callback_right(const geometry_msgs::PoseStampedConstPtr &last_msgs_right);

    void callback_button_left(const sensor_msgs::JoyConstPtr &last_button_left);

    void callback_button_right(const sensor_msgs::JoyConstPtr &last_button_right);

    void current_pose_callback_left(const geometry_msgs::PoseStampedConstPtr &msgs);

    void current_pose_callback_right(const geometry_msgs::PoseStampedConstPtr &msgs);

    void configCallback(sigma_client::PathGenerationConfig &config, uint32_t level);

    Eigen::Quaterniond scaleRotation(Eigen::Quaterniond &q_current, double scale);

    std::vector<double> toEulerAngle(Eigen::Quaterniond &q);

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
    nav_msgs::Path path_left;
    nav_msgs::Path path_right;

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
    dynamic_reconfigure::Server<sigma_client::PathGenerationConfig> server;
    dynamic_reconfigure::Server<sigma_client::PathGenerationConfig>::CallbackType f;
    int path_config = 0;
};

#endif // QUATERNIONS_H