#include "shared_controller/task_predictor.h"

Predictor::Predictor()
{
    right_ur_pose_sub = nh.subscribe("/current_pose", 1, &Predictor::rightURPoseCallback, this);
    omni_state_sub = nh.subscribe("/phantom/state", 1, &Predictor::omniStateCallback, this);
    omni_button_sub = nh.subscribe("/phantom/button", 1, &Predictor::buttonCallback, this);
}

void Predictor::rightURPoseCallback(const geometry_msgs::PoseStampedConstPtr &last_pose_right)
{
    float cur_x = last_pose_right->pose.position.x;
    float cur_y = last_pose_right->pose.position.y;
    float cur_z = last_pose_right->pose.position.z;
    delta_distance = sqrt(pow(cur_x - 0.5, 2) + pow(cur_y - 0.0, 2) + pow(cur_z - 0.1, 2));
}

void Predictor::buttonCallback(const omni_msgs::OmniButtonEventConstPtr &button_state)
{
    m_button_state = button_state->grey_button;
}

void Predictor::omniStateCallback(const omni_msgs::OmniStateConstPtr &state)
{
    vel_cur[0] = state->velocity.x;
    vel_cur[1] = state->velocity.y;
    vel_cur[2] = state->velocity.z;
}

void Predictor::updatePolicy()
{
    float max_vel = max({vel_cur[0], vel_cur[1], vel_cur[2]});
    if (delta_distance > 0.08 && max_vel > 0.2)
    {
        prob1 = 0.9;
        prob2 = 0.1;
    }
    else if (delta_distance > 0.08 || max_vel > 0.2)
    {
        prob1 = 0.6;
        prob2 = 0.4;
    }
    else
    {
        prob1 = 0.1;
        prob2 = 0.9;
    }
}
