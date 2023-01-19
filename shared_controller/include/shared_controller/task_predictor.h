#ifndef TASK_PREDICTOR_H_
#define TASK_PREDICTOR_H_

#include <vector>
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

    // void rightTouchTwistCallback(const geometry_msgs::);

public:
    ros::NodeHandle nh;
    ros::Subscriber omni_state_sub;
    ros::Subscriber omni_button_sub;
    ros::Subscriber right_ur_pose_sub;

    double prob1, prob2 = 0.0;

private:
    double alpha = 1.0;
    double delta = 0.02;
    float delta_distance = 0.0;

    vector<double> dis_positions = vector<double>(3, 0.0);
    vector<double> dis_orientation = vector<double>(3, 0.0);
    vector<float> vel_cur = vector<float>(3, 0.0);

    int m_button_state = 0;
};

#endif