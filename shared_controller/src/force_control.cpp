#include "shared_controller/force_control.h"

force_controller::force_controller()
{
    net_force_pub = nh.advertise<std_msgs::Float64MultiArray>("/touch_netforce", 1);
    ref_force_pub_right = nh.advertise<geometry_msgs::WrenchStamped>("/right/cartesian_force_controller/target_wrench", 1);
    right_force_pub = nh.advertise<geometry_msgs::WrenchStamped>("/right/cartesian_force_controller/ft_sensor_wrench", 1);

    single_force_sub = nh.subscribe("/touch_single_force", 1, &force_controller::singlePointForceCallback, this);
    pose_sub = nh.subscribe("/right/cartesian_force_controller/current_pose", 1, &force_controller::currentPoseCallback, this);
    vel_sub = nh.subscribe("/right/cartesian_force_controller/current_velocity", 1, &force_controller::currentVelocityCallback, this);

    // dynamic param
    f = boost::bind(&force_controller::configCallback, this, _1, _2);
    server.setCallback(f);

    outPose.open("/home/ur5e/Code/shared_control/data/prob001/pose.txt");
    outVel.open("/home/ur5e/Code/shared_control/data/prob001/vel.txt");
    outForce.open("/home/ur5e/Code/shared_control/data/prob001/force.txt");
}

void force_controller::calculateTorque(vector<double> &torque)
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

void force_controller::configCallback(shared_controller::trajConfig &config, uint32_t level)
{
}

void force_controller::ftSensorForcePub()
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
    right_force_pub.publish(right_sensor_force);
}

vector<float> force_controller::netForceCalculation(vector<float> &singlePointForce)
{
    for (int i = 0; i < 4; i++)
    {
        net_force[0] += singlePointForce[i];
        net_force[1] += singlePointForce[i + 4];
    }
    return net_force;
}

void force_controller::singlePointForceCallback(const std_msgs::Float64MultiArrayConstPtr &last_msg)
{
    // for (int i = 0; i < 8; i++)
    // {
    //     singlePointForce[i] = last_msg->data[i];
    //     if (i == 7)
    //     {
    //         output_singlePoint << singlePointForce[i] << endl;
    //     }
    //     else
    //     {
    //         output_singlePoint << singlePointForce[i] << "\t";
    //     }
    // }
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

void force_controller::currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &last_msgs)
{
    if (flag_right == 1)
    {
        outPose << last_msgs->pose.position.x << "\t";
        outPose << last_msgs->pose.position.y << "\t";
        outPose << last_msgs->pose.position.z << endl;
    }
}

void force_controller::currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr &last_velocity)
{
    if (flag_right == 1)
    {
        outVel << last_velocity->twist.linear.x << "\t";
        outVel << last_velocity->twist.linear.y << "\t";
        outVel << last_velocity->twist.linear.z << "\t";
        outVel << last_velocity->twist.angular.x << "\t";
        outVel << last_velocity->twist.angular.y << "\t";
        outVel << last_velocity->twist.angular.z << endl;
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "shared_controller");

    // SharedController sc;
    force_controller force_controller;

    // TouchTeleOperation tto;
    // ros::Rate loop_rate(100);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
}