#ifndef TASK_PREDICTOR_H_
#define TASK_PREDICTOR_H_

#include <vector>
#include <cmath>

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniState.h"

using namespace std;

class Predictor
{
public:
    Predictor();

    void rightURPoseCallback(const geometry_msgs::PoseStampedConstPtr &last_pose_right);

    void buttonCallback(const omni_msgs::OmniButtonEventConstPtr &button_state);

    void omniStateCallback(const omni_msgs::OmniStateConstPtr &state);

    void updatePolicy();

    void updateDistribution();

    void normalizeDistribution();

    void Console();

    // void rightTouchTwistCallback(const geometry_msgs::);

public:
    ros::NodeHandle nh;
    ros::Subscriber omni_state_sub;
    ros::Subscriber omni_button_sub;
    ros::Subscriber right_ur_pose_sub;

    double prob1, prob2 = 0.0;

private:
    int count = 0;
    double alpha = 1.0;
    double delta = 0.02;
    double weight = 1.0;
    float delta_distance = 0.0;

    vector<double> dis_positions = vector<double>(3, 0.0);
    vector<double> dis_orientation = vector<double>(3, 0.0);
    vector<double> distribution = vector<double>(2, log(0.5));
    vector<double> normalize_distribution = vector<double>(2, 0.0);
    vector<double> sum_distribution = vector<double>(2, 0.0);
    vector<float> vel_cur = vector<float>(3, 0.0);

    int grey_button_state = 0;
    int white_button_state = 0;
};

#endif