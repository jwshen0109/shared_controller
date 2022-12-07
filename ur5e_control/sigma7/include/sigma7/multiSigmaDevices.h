#ifndef MULTISIGMADEVICES_H_
#define MULTISIGMADEVICES_H_

#include "ros/ros.h"
#include <tf_conversions/tf_kdl.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>

// Sigma
#include <sigma7/dhdc.h>
#include <sigma7/drdc.h>
#include "sigma7/movingAveageFilter.h"
#include <sensor_msgs/Joy.h>
#include <vector>
#include <fstream>

#include <ur_msgs/IOStates.h>

using namespace std;

class MultiSigmaDevices
{

public:
    // Constructor with node handle and a name space for the published topics
    MultiSigmaDevices(ros::NodeHandle n);

    // Call back for the wrench subscriber. These are the forces and torques
    // that the sigma device will exert to the operator's hand.
    void WrenchCallbackLeft(const geometry_msgs::WrenchStampedConstPtr &msg);

    void WrenchCallbackRight(const geometry_msgs::WrenchStampedConstPtr &msg);

    // Fetches all the measurements (Pose, Twist, button and pedal) from the
    // device
    int ReadMeasurementsFromDevice(int id);

    // Publishing those 4 things in its name! Button and pedal will be
    // published only when they are pressed or released.
    void PublishPoseTwistButtonPedal(int id);

    // Exert wrenches if any.
    void HandleWrench(int id);

    void setLeftForceFeedback();

    void setRightForceFeedback();

    // void setForceAndTorque(int id);

private:
    // Calibrates the device using the SDK commands
    int CalibrateDevice(int id);

private:
    // the id of the device. starts from zero and increments by one if
    // another device is connected
    int id_left = 0;
    int id_right = 1;
    string left_hand;
    string right_hand;

    // sigma can simulate a button with the gripper. THat is when yoy close
    // the gripper it resists a bit at the end and and springs back when you
    // release it.
    bool enable_gripper_button = 0;

    // we can lock the orientation when the pedal is released. This is
    // useful for teleoperation
    bool lock_orient = 0;

    // the orientation matric in the locked state
    double locked_orient[3] = {0., 0., 0.};

    geometry_msgs::PoseStamped pose_msg_left, pose_msg_right;
    geometry_msgs::TwistStamped twist_msg_left, twist_msg_right;
    geometry_msgs::WrenchStamped wrench_left;
    geometry_msgs::WrenchStamped wrench_right;
    bool new_wrench_msg;
    std_msgs::Float32 gripper_angle_left, gripper_angle_right;

    // the gripper button and pedal state and their previous state
    int buttons_state_left[2], buttons_state_right[2];
    int buttons_previous_state_left[2], buttons_previous_state_right[2];

    sensor_msgs::Joy buttons_msg_left, buttons_msg_right; // two elements, 0 is gripper button, 1
    // is pedal
    int pedal_previous_state;

    // publishers and subscribers
    ros::Publisher pose_pub_left;
    ros::Publisher pose_pub_right;
    ros::Publisher twist_pub_left;
    ros::Publisher twist_pub_right;
    ros::Publisher gripper_pub_left;
    ros::Publisher gripper_pub_right;
    ros::Publisher buttons_pub_left;
    ros::Publisher buttons_pub_right;
    ros::Subscriber wrench_sub_left;
    ros::Subscriber wrench_sub_right;
};

#endif // MULTIS