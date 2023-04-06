#ifndef FORCE_CONTROL_H_
#define FORCE_CONTROL_H_

#include <vector>
#include <cmath>
#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include "shared_controller/trajConfig.h"

using namespace std;

class force_controller
{
public:
    force_controller();

    void singlePointForceCallback(const std_msgs::Float64MultiArrayConstPtr &last_msg);

    void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &last_msgs);

    void currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr &last_velocity);

    void configCallback(shared_controller::trajConfig &config, uint32_t level);

    vector<float> netForceCalculation(vector<float> &singlePointForce);

    void calculateTorque(vector<double> &torque);

    void ftSensorForcePub();
    // void rightTouchTwistCallback(const geometry_msgs::);

    dynamic_reconfigure::Server<shared_controller::trajConfig> server;
    dynamic_reconfigure::Server<shared_controller::trajConfig>::CallbackType f;

public:
    ros::NodeHandle nh;

    ros::Publisher net_force_pub;
    ros::Publisher ref_force_pub_right;
    ros::Publisher right_force_pub;

    ros::Subscriber single_force_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber vel_sub;

private:
    ofstream outPose;
    ofstream outVel;
    ofstream outForce;

    vector<float> net_force = vector<float>(2, 0.0);
    vector<float> singlePointForce = vector<float>(6, 0.0);

    int flag_left = 0;
    int flag_right = 0;
};

#endif