#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <ur5e_test/referenceConfig.h>
#include <vector>

using namespace std;

ros::Publisher net_force_pub;
ros::Publisher left_force_pub;
ros::Publisher right_force_pub;

// output data
ofstream output_torque;
ofstream output_singlePoint;
ofstream output_pose;
ofstream output_vel;
ofstream output_joint;

vector<float> net_force(2, 0.0);
vector<float> singlePointForce(8, 0.0);

int flag_left = 0;
int flag_right = 0;

void calculateTorque(vector<double> &torque)
{

    if (net_force[1] >= 0.04)
    {
        std_msgs::Float64MultiArray location_msg;
        double delta_x = (singlePointForce[5] + singlePointForce[7] - singlePointForce[4] - singlePointForce[6]) * 5.5 / net_force[1];
        double delta_y = (singlePointForce[4] + singlePointForce[7] - singlePointForce[6] - singlePointForce[5]) * 2.8 / net_force[1];
        torque[0] = net_force[1] * delta_y;
        torque[1] = net_force[1] * delta_x;
    }
    if (net_force[0] >= 0.04)
    {
        double delta_x_l = (singlePointForce[0] + singlePointForce[2] - singlePointForce[1] - singlePointForce[3]) * 5.5 / net_force[0];
        double delta_y_l = (singlePointForce[0] + singlePointForce[3] - singlePointForce[1] - singlePointForce[2]) * 2.8 / net_force[0];
        torque[2] = net_force[0] * delta_y_l;
        torque[3] = net_force[0] * delta_x_l;
    }
}

void ftSensorForcePub()
{
    double threshold = 1.0;
    geometry_msgs::WrenchStamped left_sensor_force;
    geometry_msgs::WrenchStamped right_sensor_force;
    left_sensor_force.header.stamp = ros::Time::now();
    left_sensor_force.header.frame_id = "retractor";
    left_sensor_force.wrench.force.x = net_force[0];
    left_sensor_force.wrench.force.y = 0.0;
    left_sensor_force.wrench.force.z = 0.0;
    left_sensor_force.wrench.torque.x = 0.0;
    left_sensor_force.wrench.torque.y = 0.0;
    left_sensor_force.wrench.torque.z = 0.0;

    right_sensor_force.header.stamp = ros::Time::now();
    right_sensor_force.header.frame_id = "retractor";
    right_sensor_force.wrench.force.x = net_force[1];
    right_sensor_force.wrench.force.y = 0.0;
    right_sensor_force.wrench.force.z = 0.0;
    right_sensor_force.wrench.torque.x = 0.0;
    right_sensor_force.wrench.torque.y = 0.0;
    right_sensor_force.wrench.torque.z = 0.0;

    vector<double> torque(4, 0.0);
    calculateTorque(torque);

    output_torque << torque[0] << "\t";
    output_torque << torque[1] << endl;

    // if (abs(torque[0]) >= threshold)
    // {
    //     right_sensor_force.wrench.torque.z = torque[0];
    // }
    // if (abs(torque[1]) >= threshold)
    // {
    //     right_sensor_force.wrench.torque.y = -torque[1] / 2;
    //     // ROS_INFO("torque1:%f", torque[1]);
    // }
    // if (abs(torque[2]) >= threshold)
    // {
    //     left_sensor_force.wrench.torque.z = torque[2];
    // }
    // if (abs(torque[3]) >= threshold)
    // {
    //     left_sensor_force.wrench.torque.y = -torque[3] / 2;
    //     // ROS_INFO("torque1:%f", torque[1]);
    // }

    left_force_pub.publish(left_sensor_force);
    right_force_pub.publish(right_sensor_force);
}

vector<float> netForceCalculation(vector<float> &singlePointForce)
{
    for (int i = 0; i < 4; i++)
    {
        net_force[0] += singlePointForce[i];
        net_force[1] += singlePointForce[i + 4];
    }
    return net_force;
}

void singlePointForceCallback(const std_msgs::Float64MultiArrayConstPtr &last_msg)
{
    for (int i = 0; i < 8; i++)
    {
        singlePointForce[i] = last_msg->data[i];
        if (i == 7)
        {
            output_singlePoint << singlePointForce[i] << endl;
        }
        else
        {
            output_singlePoint << singlePointForce[i] << "\t";
        }
    }
    net_force[0] = net_force[1] = 0;
    net_force = netForceCalculation(singlePointForce);
    if (net_force[0] >= 0.95)
    {
        flag_left = 1;
    }
    else
    {
        flag_left = 0;
    }
    if (net_force[1] >= 0.95)
    {
        flag_right = 1;
    }
    else
    {
        flag_right = 0;
    }
    std_msgs::Float64MultiArray netforce_msgs;
    netforce_msgs.data.push_back(net_force[0]);
    netforce_msgs.data.push_back(net_force[1]);

    net_force_pub.publish(netforce_msgs);
}

void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &last_msgs)
{
    if (flag_right == 1)
    {
        output_pose << last_msgs->pose.position.x << "\t";
        output_pose << last_msgs->pose.position.y << "\t";
        output_pose << last_msgs->pose.position.z << "\t";
        output_pose << last_msgs->pose.orientation.x << "\t";
        output_pose << last_msgs->pose.orientation.y << "\t";
        output_pose << last_msgs->pose.orientation.z << "\t";
        output_pose << last_msgs->pose.orientation.w << endl;
    }
}

void currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr &last_velocity)
{
    if (flag_right == 1)
    {
        output_vel << last_velocity->twist.linear.x << "\t";
        output_vel << last_velocity->twist.linear.y << "\t";
        output_vel << last_velocity->twist.linear.z << "\t";
        output_vel << last_velocity->twist.angular.x << "\t";
        output_vel << last_velocity->twist.angular.y << "\t";
        output_vel << last_velocity->twist.angular.z << endl;
    }
}

void jointCallback(const sensor_msgs::JointStateConstPtr &last_joint)
{
    if (flag_right == 1)
    {
        for (int i = 0; i < 6; i++)
        {
            output_joint << last_joint->position[i] << "\t";
        }
        for (int i = 0; i < 5; i++)
        {
            output_joint << last_joint->velocity[i] << "\t";
        }
        output_joint << last_joint->velocity[5] << endl;
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_threshold");
    ros::NodeHandle nh;

    // subscribe force data from serial com
    ros::Subscriber single_force_sub = nh.subscribe("/touch_single_force", 1, &singlePointForceCallback);
    net_force_pub = nh.advertise<std_msgs::Float64MultiArray>("/touch_netforce", 1);

    // subscribe UR retractor position velocity and joint states
    ros::Subscriber pose_sub = nh.subscribe("/right/cartesian_force_controller/current_pose", 1, &currentPoseCallback);
    ros::Subscriber velocity_sub = nh.subscribe("/right/cartesian_force_controller/current_velocity", 1, &currentVelocityCallback);
    ros::Subscriber joint_state_sub = nh.subscribe("/right/joint_states", 1, &jointCallback);

    // publish reference force to special topic
    ros::Publisher ref_force_pub_left = nh.advertise<geometry_msgs::WrenchStamped>("/left/cartesian_force_controller/target_wrench", 1);
    ros::Publisher ref_force_pub_right = nh.advertise<geometry_msgs::WrenchStamped>("/right/cartesian_force_controller/target_wrench", 1);

    // publish force to ft_sensor topic
    left_force_pub = nh.advertise<geometry_msgs::WrenchStamped>("/left/cartesian_force_controller/ft_sensor_wrench", 1);
    right_force_pub = nh.advertise<geometry_msgs::WrenchStamped>("/right/cartesian_force_controller/ft_sensor_wrench", 1);

    ros::Rate loop_rate(100);
    geometry_msgs::WrenchStamped ref_force_left;
    geometry_msgs::WrenchStamped ref_force_right;

    ref_force_left.header.stamp = ros::Time::now();
    ref_force_left.header.frame_id = "retractor";
    ref_force_left.wrench.force.x = -1.0;
    ref_force_left.wrench.force.y = 0.0;
    ref_force_left.wrench.force.z = 0.0;
    ref_force_left.wrench.torque.x = 0.0;
    ref_force_left.wrench.torque.y = 0.0;
    ref_force_left.wrench.torque.z = 0.0;

    ref_force_right.header.stamp = ros::Time::now();
    ref_force_right.header.frame_id = "retractor";
    ref_force_right.wrench = ref_force_left.wrench;

    // dynamic_reconfigure callback setting;
    // dynamic_reconfigure::Server<ur5e_test::referenceConfig> server;
    // dynamic_reconfigure::Server<ur5e_test::referenceConfig>::CallbackType f;
    // f = boost::bind(&configCallback, _1, _2);
    // server.setCallback(f);

    output_torque.open("/home/ur5e/Code/force_control/data/torque.txt");
    output_singlePoint.open("/home/ur5e/Code/force_control/data/singlePointForce.txt");
    output_pose.open("/home/ur5e/Code/force_control/data/pose.txt");
    output_vel.open("/home/ur5e/Code/force_control/data/vel.txt");
    output_joint.open("/home/ur5e/Code/force_control/data/joint.txt");

    while (ros::ok())
    {

        ref_force_pub_left.publish(ref_force_left);
        ref_force_pub_right.publish(ref_force_right);
        ftSensorForcePub();
        ros::spinOnce();
        loop_rate.sleep();
    }
}