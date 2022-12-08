#include "shared_controller/shared_controller.h"

SharedController::SharedController()
{

    ft_force_pub = nh.advertise<geometry_msgs::WrenchStamped>("/right/cartesian_force_controller/ft_sensor_wrench", 1);

    reference_force_pub = nh.advertise<geometry_msgs::WrenchStamped>("/right/cartesian_force_controller/target_wrench", 1);

    sigma_twist_sub = nh.subscribe("/sigma7/sigma0/twist", 1, &SharedController::twistCallback, this);

    sigma_button_sub = nh.subscribe("/sigma7/sigma0/buttons", 1, &SharedController::buttonCallback, this);

    f = boost::bind(&SharedController::configCallback, this, _1, _2);
    server.setCallback(f);
}

void SharedController::twistCallback(const geometry_msgs::TwistStampedConstPtr &last_msg)
{

    if (button_states[0])
    {
        delta_twist[0] = last_msg->twist.linear.x;
        delta_twist[1] = last_msg->twist.linear.y;
        delta_twist[2] = last_msg->twist.linear.z;
        delta_twist[3] = last_msg->twist.angular.x;
        delta_twist[4] = last_msg->twist.angular.y;
        delta_twist[5] = last_msg->twist.angular.z;
    }
    else
    {
        for (int i = 0; i < 6; i++)
        {
            delta_twist[i] = 0.0;
        }
    }

    geometry_msgs::WrenchStamped reference_force;
    reference_force.header.stamp = ros::Time::now();
    reference_force.header.frame_id = "retractor";
    reference_force.wrench.force.x = -drp.ref_force_x;
    reference_force.wrench.force.y = 0.0;
    reference_force.wrench.force.z = 0.0;
    reference_force.wrench.torque.x = 0.0;
    reference_force.wrench.torque.y = 0.0;
    reference_force.wrench.torque.z = 0.0;
    reference_force_pub.publish(reference_force);
}

void SharedController::buttonCallback(const sensor_msgs::JoyConstPtr &last_state)
{
    button_states[0] = last_state->buttons[0];
}

void SharedController::publishFtSensorForce()
{
    geometry_msgs::WrenchStamped ft_force;
    ft_force.header.stamp = ros::Time::now();
    ft_force.header.frame_id = "retractor";
    ft_force.wrench.force.x = delta_twist[2] * drp.scale;
    ft_force.wrench.force.y = delta_twist[1] * drp.scale;
    ft_force.wrench.force.z = delta_twist[0] * drp.scale;
    ft_force.wrench.torque.x = delta_twist[5] * drp.scale;
    ft_force.wrench.torque.y = delta_twist[4] * drp.scale;
    ft_force.wrench.torque.z = delta_twist[3] * drp.scale;
    ft_force_pub.publish(ft_force);
}

void SharedController::configCallback(shared_controller::commandConfig &config, uint32_t level)
{
    init = config.init;
    // goal = config.goal_select;
    drp.ref_force_x = config.ref_force_x;
    drp.scale = config.scale;
}

TeleManipulation::TeleManipulation()
{
    // topic to cartesian controllers
    target_pub_left = nh.advertise<geometry_msgs::PoseStamped>("/left/cartesian_motion_controller/target_frame", 1);
    target_pub_right = nh.advertise<geometry_msgs::PoseStamped>("/right/cartesian_motion_controller/target_frame", 1);
    // path_pub_left = nh.advertise<nav_msgs::Path>("/left/path", 1);
    // path_pub_right = nh.advertise<nav_msgs::Path>("/right/path", 1);
    // subscribe from sigma7 devices
    // pose
    sub_sigma_left = nh.subscribe("/sigma7/sigma1/pose", 1, &TeleManipulation::callback_left, this);
    sub_sigma_right = nh.subscribe("/sigma7/sigma0/pose", 1, &TeleManipulation::callback_right, this);
    // buttons
    sub_sigma_button_left = nh.subscribe("/sigma7/sigma1/buttons", 1, &TeleManipulation::callback_button_left, this);
    sub_sigma_button_right = nh.subscribe("/sigma7/sigma0/buttons", 1, &TeleManipulation::callback_button_right, this);

    // subscribe current state
    sub_current_pose_left = nh.subscribe("/left/cartesian_motion_controller/current_pose", 1, &TeleManipulation::current_pose_callback_left, this);
    sub_current_pose_right = nh.subscribe("/right/cartesian_motion_controller/current_pose", 1, &TeleManipulation::current_pose_callback_right, this);

    // path config
    // f = boost::bind(&TeleManipulation::configCallback, this, _1, _2);
    // server.setCallback(f);
    target_cylinder = new Cylinder(0, 0.4, 0.1);
    p_current = new Point(0, 0, 0);
}

void TeleManipulation::current_pose_callback_left(const geometry_msgs::PoseStampedConstPtr &msgs)
{
    current_pose_left.pose = msgs->pose;
    if (first_flag_left == 0)
    {
        first_flag_left = 2;
    }
}

void TeleManipulation::current_pose_callback_right(const geometry_msgs::PoseStampedConstPtr &msgs)
{
    current_pose_right.pose = msgs->pose;
    if (first_flag_right == 0)
    {
        first_flag_right = 2;
    }
    p_current->x = current_pose_right.pose.position.x;
    p_current->y = current_pose_right.pose.position.y;
    p_current->z = current_pose_right.pose.position.z;
    vf.PublishVirtualForce(*p_current, *target_cylinder);
}

// void TeleManipulation::configCallback(sigma_client::PathGenerationConfig &config, uint32_t level)
// {
//   path_config = config.Path_command;
//   // ROS_INFO("here...");
// }

void TeleManipulation::callback_left(const geometry_msgs::PoseStampedConstPtr &last_msgs_left)
{

    // sigma transform axis--z -90degree
    q_transform_left.x() = 0.0;
    q_transform_left.y() = 0.0;
    q_transform_left.z() = -0.7071068;
    q_transform_left.w() = 0.7071068;

    // current sigma orientation
    q_sigma_left.x() = last_msgs_left->pose.orientation.x;
    q_sigma_left.y() = last_msgs_left->pose.orientation.y;
    q_sigma_left.z() = last_msgs_left->pose.orientation.z;
    q_sigma_left.w() = last_msgs_left->pose.orientation.w;

    // transform sigma coordinates: z -90.
    q_target_left = q_transform_left * q_sigma_left;

    geometry_msgs::PoseStamped target_pose_left;
    target_pose_left.header.stamp = ros::Time::now();
    target_pose_left.header.frame_id = "left_base_link";

    // delta position of sigma7
    delta_position[0] = last_msgs_left->pose.position.x - last_sigma_left.pose.position.x;
    delta_position[1] = last_msgs_left->pose.position.y - last_sigma_left.pose.position.y;
    delta_position[2] = last_msgs_left->pose.position.z - last_sigma_left.pose.position.z;

    // delta orientation of sigma7
    q_cur_left.x() = q_target_left.x();
    q_cur_left.y() = q_target_left.y();
    q_cur_left.z() = q_target_left.z();
    q_cur_left.w() = q_target_left.w();
    q_last_left.x() = last_sigma_left.pose.orientation.x;
    q_last_left.y() = last_sigma_left.pose.orientation.y;
    q_last_left.z() = last_sigma_left.pose.orientation.z;
    q_last_left.w() = last_sigma_left.pose.orientation.w;

    delta_q_left = q_cur_left * q_last_left.conjugate();

    // init state
    if (first_flag_left == 2 && button_left == 0)
    {
        // if(first_flag_left == 0 && button_left == 0){
        target_pose_left.pose.position.x = 0.4;
        target_pose_left.pose.position.y = 0.0;
        target_pose_left.pose.position.z = 0.4;
        target_pose_left.pose.orientation.x = 0.0;
        target_pose_left.pose.orientation.y = 1.0;
        target_pose_left.pose.orientation.z = 0.0;
        target_pose_left.pose.orientation.w = 0.0;
        // target_pose_left.pose.position.x = current_pose_left.pose.position.x;
        // target_pose_left.pose.position.y = current_pose_left.pose.position.y;
        // target_pose_left.pose.position.z = current_pose_left.pose.position.z;
        // target_pose_left.pose.orientation.x = current_pose_left.pose.orientation.x;
        // target_pose_left.pose.orientation.y = current_pose_left.pose.orientation.y;
        // target_pose_left.pose.orientation.z = current_pose_left.pose.orientation.z;
        // target_pose_left.pose.orientation.w = current_pose_left.pose.orientation.w;
        target_pub_left.publish(target_pose_left);
        last_pose_left.pose = target_pose_left.pose;
        first_flag_left = 1;
    }
    else if (first_flag_left == 1 && button_left == 1)
    { // maste-slave mode

        ur_q_cur_left.x() = last_pose_left.pose.orientation.x;
        ur_q_cur_left.y() = last_pose_left.pose.orientation.y;
        ur_q_cur_left.z() = last_pose_left.pose.orientation.z;
        ur_q_cur_left.w() = last_pose_left.pose.orientation.w;

        ur_q_target_left = delta_q_left * ur_q_cur_left;

        target_pose_left.pose.position.x = last_pose_left.pose.position.x + delta_position[1];
        target_pose_left.pose.position.y = last_pose_left.pose.position.y - delta_position[0];
        target_pose_left.pose.position.z = last_pose_left.pose.position.z + delta_position[2];

        // target_pose_left.pose.orientation.x = q_target.x();
        // target_pose_left.pose.orientation.y = q_target.y();
        // target_pose_left.pose.orientation.z = q_target.z();
        // target_pose_left.pose.orientation.w = q_target.w();

        // target_pose_left.pose.orientation.x = last_msgs_left->pose.orientation.w;
        // target_pose_left.pose.orientation.y = last_msgs_left->pose.orientation.z;
        // target_pose_left.pose.orientation.z = last_msgs_left->pose.orientation.x;
        // target_pose_left.pose.orientation.w = -last_msgs_left->pose.orientation.y;
        // ur_q_target_left_scale = scaleRotation(ur_q_target_left, 1.0);
        target_pose_left.pose.orientation.x = ur_q_target_left.x();
        target_pose_left.pose.orientation.y = ur_q_target_left.y();
        target_pose_left.pose.orientation.z = ur_q_target_left.z();
        target_pose_left.pose.orientation.w = ur_q_target_left.w();

        target_pub_left.publish(target_pose_left);

        // publish target trajectory path
        // path_left.header.stamp = ros::Time::now();
        // path_left.header.frame_id = "left_base_link";
        // if (path_config == 1)
        // {
        //     path_left.poses.push_back(target_pose_left);
        //     path_pub_left.publish(path_left);
        // }
        // if (path_config == 2)
        // {
        //     path_left.poses.clear();
        // }

        // record last time target poses
        last_pose_left.pose = target_pose_left.pose;
    }
    else
    { // stop master-slave mode
        last_pose_left.header.stamp = ros::Time::now();
        last_pose_left.header.frame_id = "left_base_link";
        target_pub_left.publish(last_pose_left);
    }

    // update current pose of sigma7
    last_sigma_left.pose.position = last_msgs_left->pose.position;
    last_sigma_left.pose.orientation.x = q_target_left.x();
    last_sigma_left.pose.orientation.y = q_target_left.y();
    last_sigma_left.pose.orientation.z = q_target_left.z();
    last_sigma_left.pose.orientation.w = q_target_left.w();
}

void TeleManipulation::callback_right(const geometry_msgs::PoseStampedConstPtr &last_msgs_right)
{

    q_transform_right.x() = 0.0;
    q_transform_right.y() = 0.0;
    q_transform_right.z() = 0.7071068;
    q_transform_right.w() = 0.7071068;

    // current sigma orientation
    q_sigma_right.x() = last_msgs_right->pose.orientation.x;
    q_sigma_right.y() = last_msgs_right->pose.orientation.y;
    q_sigma_right.z() = last_msgs_right->pose.orientation.z;
    q_sigma_right.w() = last_msgs_right->pose.orientation.w;

    q_target_right = q_transform_right * q_sigma_right;

    geometry_msgs::PoseStamped target_pose_right;

    target_pose_right.header.stamp = ros::Time::now();
    target_pose_right.header.frame_id = "right_base_link";

    delta_position[3] = last_msgs_right->pose.position.x - last_sigma_right.pose.position.x;
    delta_position[4] = last_msgs_right->pose.position.y - last_sigma_right.pose.position.y;
    delta_position[5] = last_msgs_right->pose.position.z - last_sigma_right.pose.position.z;

    q_cur_right.x() = q_target_right.x();
    q_cur_right.y() = q_target_right.y();
    q_cur_right.z() = q_target_right.z();
    q_cur_right.w() = q_target_right.w();
    q_last_right.x() = last_sigma_right.pose.orientation.x;
    q_last_right.y() = last_sigma_right.pose.orientation.y;
    q_last_right.z() = last_sigma_right.pose.orientation.z;
    q_last_right.w() = last_sigma_right.pose.orientation.w;

    delta_q_right = q_cur_right * q_last_right.conjugate();

    if (first_flag_right == 2 && button_right == 0)
    {

        target_pose_right.pose.position.x = 0.4;
        target_pose_right.pose.position.y = 0.0;
        target_pose_right.pose.position.z = 0.4;
        target_pose_right.pose.orientation.x = 0.0;
        target_pose_right.pose.orientation.y = 1.0;
        target_pose_right.pose.orientation.z = 0.0;
        target_pose_right.pose.orientation.w = 0.0;
        // target_pose_right.pose.position.x = current_pose_right.pose.position.x;
        // target_pose_right.pose.position.y = current_pose_right.pose.position.y;
        // target_pose_right.pose.position.z = current_pose_right.pose.position.z;
        // target_pose_right.pose.orientation.x = current_pose_right.pose.orientation.x;
        // target_pose_right.pose.orientation.y = current_pose_right.pose.orientation.y;
        // target_pose_right.pose.orientation.z = current_pose_right.pose.orientation.z;
        // target_pose_right.pose.orientation.w = current_pose_right.pose.orientation.w;
        target_pub_right.publish(target_pose_right);
        last_pose_right.pose = target_pose_right.pose;
        first_flag_right = 1;
    }
    else if (first_flag_right == 1 && button_right == 1)
    {

        ur_q_cur_right.x() = last_pose_right.pose.orientation.x;
        ur_q_cur_right.y() = last_pose_right.pose.orientation.y;
        ur_q_cur_right.z() = last_pose_right.pose.orientation.z;
        ur_q_cur_right.w() = last_pose_right.pose.orientation.w;

        ur_q_target_right = delta_q_right * ur_q_cur_right;

        target_pose_right.pose.position.x = last_pose_right.pose.position.x - delta_position[4];
        target_pose_right.pose.position.y = last_pose_right.pose.position.y + delta_position[3];
        target_pose_right.pose.position.z = last_pose_right.pose.position.z + delta_position[5];
        // target_pose_right.pose.orientation.x = -last_msgs_right->pose.orientation.w;
        // target_pose_right.pose.orientation.y = -last_msgs_right->pose.orientation.z;
        // target_pose_right.pose.orientation.z = last_msgs_right->pose.orientation.x;
        // target_pose_right.pose.orientation.w = -last_msgs_right->pose.orientation.y;
        target_pose_right.pose.orientation.x = ur_q_target_right.x();
        target_pose_right.pose.orientation.y = ur_q_target_right.y();
        target_pose_right.pose.orientation.z = ur_q_target_right.z();
        target_pose_right.pose.orientation.w = ur_q_target_right.w();

        target_pub_right.publish(target_pose_right);
        last_pose_right.pose = target_pose_right.pose;
    }
    else
    {

        last_pose_right.header.stamp = ros::Time::now();
        last_pose_right.header.frame_id = "right_base_link";
        target_pub_right.publish(last_pose_right);
    }

    last_sigma_right.pose.position = last_msgs_right->pose.position;
    last_sigma_right.pose.orientation.x = q_target_right.x();
    last_sigma_right.pose.orientation.y = q_target_right.y();
    last_sigma_right.pose.orientation.z = q_target_right.z();
    last_sigma_right.pose.orientation.w = q_target_right.w();
}

void TeleManipulation::callback_button_left(const sensor_msgs::JoyConstPtr &last_button_left)
{
    button_left = last_button_left->buttons[0];
}

void TeleManipulation::callback_button_right(const sensor_msgs::JoyConstPtr &last_button_right)
{
    button_right = last_button_right->buttons[0];
}

Eigen::Quaterniond TeleManipulation::scaleRotation(Eigen::Quaterniond &q_current, double scale)
{
    Eigen::Quaterniond scale_q;
    std::vector<double> eulerAngles = toEulerAngle(q_current);
    for (int i = 0; i < 3; i++)
    {
        eulerAngles[0] *= scale;
        eulerAngles[1] *= scale;
        eulerAngles[2] *= scale;
    }
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngles[0], ::Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngles[1], ::Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngles[2], ::Eigen::Vector3d::UnitZ()));
    scale_q = rollAngle * pitchAngle * yawAngle;

    return scale_q;
}

vector<double> TeleManipulation::toEulerAngle(Eigen::Quaterniond &q)
{
    std::vector<double> eulerAngles(3);
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    eulerAngles[0] = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        eulerAngles[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        eulerAngles[1] = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    eulerAngles[2] = atan2(siny_cosp, cosy_cosp);

    return eulerAngles;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "shared_controller");

    SharedController sc;
    ros::Rate loop_rate(100);
    // ros::AsyncSpinner spinner(2);
    // spinner.start();
    // ros::waitForShutdown();
    while (ros::ok())
    {
        sc.publishFtSensorForce();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}