#ifndef SHARED_CONTROLLER_H_
#define SHARED_CONTROLLER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <shared_controller/commandConfig.h>
#include "omni_msgs/OmniButtonEvent.h"
#include "ur_msgs/IOStates.h"
#include "shared_controller/task_predictor.h"

#include <termio.h>
#include <iostream>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <fstream>

#include "shared_controller/artificial_potential_field.h"

#define PI 3.141592653
#define EPSILON 0.00005
constexpr auto e = 2.71828;

using namespace std;

struct dynamic_reconfigure_params
{
    float delta_step = 0.0;
    double scale = 0.1;
    double scale1 = 2;
    float eta_p = 0.0;
    float eta_v = 0.0;
    float kp = 0.0;
    float kd = 0.0;
    float auto_delta = 0.0;
    float force_k = 1.0;
    float angle = 15.0;

    bool apf = false;
    bool blend = false;
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

public:
    dynamic_reconfigure_params drp;

private:
    ros::Publisher ft_force_pub;
    ros::Publisher reference_force_pub;
    ros::Subscriber sigma_twist_sub;
    ros::Subscriber sigma_button_sub;

    // dynamic_reconfigure::Server<shared_controller::commandConfig> server;
    // dynamic_reconfigure::Server<shared_controller::commandConfig>::CallbackType f;

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

    void callback_vel_left(const geometry_msgs::TwistStampedConstPtr &last_vel);

    void callback_vel_right(const geometry_msgs::TwistStampedConstPtr &last_vel);

    void callback_button_left(const sensor_msgs::JoyConstPtr &last_button_left);

    void callback_button_right(const sensor_msgs::JoyConstPtr &last_button_right);

    void current_pose_callback_left(const geometry_msgs::PoseStampedConstPtr &msgs);

    void current_pose_callback_right(const geometry_msgs::PoseStampedConstPtr &msgs);

    void current_velocity_callback_left(const geometry_msgs::TwistStampedConstPtr &last_velocity);

    void current_velocity_callback_right(const geometry_msgs::TwistStampedConstPtr &last_velocity);

    void configCallback(shared_controller::commandConfig &config, uint32_t level);

    void singlePointForceCallback(const std_msgs::Float64MultiArrayConstPtr &last_msg);

    void newRetractorCallback(const std_msgs::Float64MultiArrayConstPtr &last_msgs);

    void IOStateCallback(const ur_msgs::IOStatesConstPtr &state);

    vector<float> forceToMotionControl(float retractor_nForce);

    void forceFeedback(float fz);

    float Fedge(vector<float> &retractor_cur, vector<float> &vel, int id);

    void updateProbability();

    vector<float> calculateRetractorDis(vector<float> &leftRetractor, vector<float> &rightRetractor);

    // Eigen::Quaterniond scaleRotation(Eigen::Quaterniond &q_current, double scale);

    vector<float> QuaternionToEulerAngle(geometry_msgs::PoseStamped &pose);

public:
    // for APF
    APF apf;
    // forceVector forceVector;
    // Point2D *retractor_cur;
    // Point2D *p0;
    vector<float> xyForce = vector<float>(2, 0.0f);

    dynamic_reconfigure_params drp;

    // Cylinder *target_cylinder;
    // VirtualFixture vf;
    // SharedController sc;
    // Predictor predictor;

    // shared control
    vector<vector<float>> probability = vector<vector<float>>(4, vector<float>(4, 0.0));
    vector<float> auto_delta_position = vector<float>(3, 0.0f);

private:
    vector<float> netForceCalculation(vector<float> &singlePointForce);

private:
    vector<float> retractor_spForce = vector<float>(8, 0.0f);
    vector<float> retractor_nForce = vector<float>(2, 0.0f);
    vector<float> leftRetractor_Coordians = vector<float>(3, 0.0f);
    vector<float> rightRetractor_Coordians = vector<float>(3, 0.0f);
    float relativeDistance = 0.0f;

    // force control
    // float kd = 1.0f;
    // float delta_dis = 0.1f;

private:
    ros::NodeHandle nh;
    ros::Publisher target_pub_left;
    ros::Publisher target_pub_right;
    ros::Publisher path_pub_left;
    ros::Publisher path_pub_right;
    ros::Publisher netforce_pub;
    ros::Publisher vfForce_pub;
    ros::Publisher dynamicParam_pub;

    ros::ServiceClient head_client;

    // poses & buttons subscriber
    ros::Subscriber sub_sigma_left;
    ros::Subscriber sub_sigma_right;
    ros::Subscriber sub_sigma_vel_left;
    ros::Subscriber sub_sigma_vel_right;
    ros::Subscriber sub_sigma_button_left;
    ros::Subscriber sub_sigma_button_right;
    ros::Subscriber sub_current_pose_left;
    ros::Subscriber sub_current_pose_right;
    ros::Subscriber sub_current_velocity_left;
    ros::Subscriber sub_current_velocity_right;
    ros::Subscriber single_point_force_sub;
    ros::Subscriber newretractor_force_sub;
    ros::Subscriber IOState_sub;

    geometry_msgs::PoseStamped last_pose_left;
    geometry_msgs::PoseStamped last_sigma_left;
    geometry_msgs::PoseStamped last_pose_right;
    geometry_msgs::PoseStamped last_sigma_right;
    geometry_msgs::PoseStamped current_pose_left;
    geometry_msgs::PoseStamped current_pose_right;
    geometry_msgs::TwistStamped current_twist_left;
    geometry_msgs::TwistStamped current_twist_right;

    // 0,1,2 for left ur; 3,4,5 for right ur
    vector<float> delta_position = vector<float>(6, 0.0);

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

    // iostates
    int iostates = 0;

    // init state
    int first_flag_left = 0;
    int first_flag_right = 0;

    // dynamic_reconfigure config
    dynamic_reconfigure::Server<shared_controller::commandConfig> server;
    dynamic_reconfigure::Server<shared_controller::commandConfig>::CallbackType f;
    // int path_config = 0;

    // for velocity angle
    float lambda = 0.0174f;
    float beta = 1.5f;
    vector<float> velocity_left = vector<float>(3, 0.0);
    vector<float> velocity_right = vector<float>(3, 0.0);
    vector<float> angle_left = vector<float>(3, 0.0);
    vector<float> angle_right = vector<float>(3, 0.0);

    Eigen::Matrix4d T_Markov = Eigen::Matrix4d::Identity();

    // APF param
    float eta_att = 0.3;
    float eta_rep = 0.02;
    float delta_t = 0.005;

    // retractor size
    float retractor_width = 15;
    float retractor_height = 35;
    // edge size
    float edge_width = 35;
    float edge_height = 55;
    float last_retractor_force = 0.0;

    int apf_flag = 0;
    float step = 0.002;
    vector<float> p0 = vector<float>(3, 0.0);
    vector<float> delta_last = vector<float>(3, 0.0);
    vector<float> retractor_cur = vector<float>(3, 0.0);
    vector<float> vel = vector<float>(3, 0.0);
    vector<float> last_rightmaster = vector<float>(3, 0.0);
    vector<float> net_newforce = vector<float>(2, 0.0);
    vector<float> net_newTorque = vector<float>(2, 0.0);

    std::ofstream outFile;
    std::ofstream outVel;
    std::ofstream outP_foce;
    std::ofstream outforce;
    std::ofstream output_rm;
    std::ofstream output_rs;
    std::ofstream output_angle;
    std::ofstream output_re_vel;
};

class TouchTeleOperation
{
public:
    TouchTeleOperation();

    void PoseCallback(const geometry_msgs::PoseStampedConstPtr &last_pose);

    void buttonCallback(const omni_msgs::OmniButtonEventConstPtr &button_state);

public:
    Predictor predictor_tmp;

private:
    ros::NodeHandle nh;
    ros::Publisher target_pose_pub;
    ros::Subscriber omni_pose_sub;
    ros::Subscriber button;

    geometry_msgs::PoseStamped last_pose;
    geometry_msgs::PoseStamped last_omni;

    vector<double> delta_position_touch = {0, 0, 0};
    Eigen::Quaterniond delta_q;
    Eigen::Quaterniond q_cur;
    Eigen::Quaterniond q_last;
    Eigen::Quaterniond ur_q_cur;
    Eigen::Quaterniond ur_q_target;

    Eigen::Quaterniond q_omni;
    Eigen::Quaterniond q_target;
    Eigen::Quaterniond q_transform;
    Eigen::Quaterniond q_transform1;

    int grey_button = 0;
    int white_button = 0;
    int first_flag = 0;
};

#endif //