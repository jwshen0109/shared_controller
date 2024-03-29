#include "shared_controller/shared_controller.h"

SharedController::SharedController()
{

    ft_force_pub = nh.advertise<geometry_msgs::WrenchStamped>("/right/cartesian_force_controller/ft_sensor_wrench", 1);

    reference_force_pub = nh.advertise<geometry_msgs::WrenchStamped>("/right/cartesian_force_controller/target_wrench", 1);

    sigma_twist_sub = nh.subscribe("/sigma7/sigma0/twist", 1, &SharedController::twistCallback, this);

    sigma_button_sub = nh.subscribe("/sigma7/sigma0/buttons", 1, &SharedController::buttonCallback, this);

    // f = boost::bind(&SharedController::configCallback, this, _1, _2);
    // server.setCallback(f);
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
    reference_force.wrench.force.x = -1.0;
    reference_force.wrench.force.y = 0.0;
    reference_force.wrench.force.z = 0.0;
    reference_force.wrench.torque.x = 0.0;
    reference_force.wrench.torque.y = 0.0;
    reference_force.wrench.torque.z = 0.0;
    reference_force_pub.publish(reference_force);
    publishFtSensorForce();
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
    // init = config.init;
    // // goal = config.goal_select;
    // drp.ref_force_x = config.ref_force_x;
    // drp.scale = config.scale;
    // drp.eta_p = config.eta_p;
    // drp.eta_v = config.eta_v;
    // drp.radius = config.cylinder_radius;
}

TeleOperation::TeleOperation()
{
    // topic to cartesian controllers
    target_pub_left = nh.advertise<geometry_msgs::PoseStamped>("/left/cartesian_motion_controller/target_frame", 1);
    target_pub_right = nh.advertise<geometry_msgs::PoseStamped>("/right/cartesian_motion_controller/target_frame", 1);
    netforce_pub = nh.advertise<std_msgs::Float64MultiArray>("/touch_netforce", 1);
    vfForce_pub = nh.advertise<geometry_msgs::WrenchStamped>("/sigma0/force_feedback", 1);
    dynamicParam_pub = nh.advertise<std_msgs::Float64MultiArray>("/dynamic_param", 1);
    // path_pub_left = nh.advertise<nav_msgs::Path>("/left/path", 1);
    // path_pub_right = nh.advertise<nav_msgs::Path>("/right/path", 1);
    // subscribe from sigma7 devices
    // pose
    sub_sigma_left = nh.subscribe("/sigma7/sigma1/pose", 1, &TeleOperation::callback_left, this);
    sub_sigma_right = nh.subscribe("/sigma7/sigma0/pose", 1, &TeleOperation::callback_right, this);
    sub_sigma_vel_left = nh.subscribe("/sigma7/sigma1/twist", 1, &TeleOperation::callback_vel_left, this);
    sub_sigma_vel_right = nh.subscribe("/sigma7/sigma1/twist", 1, &TeleOperation::callback_vel_right, this);
    // buttons
    sub_sigma_button_left = nh.subscribe("/sigma7/sigma1/buttons", 1, &TeleOperation::callback_button_left, this);
    sub_sigma_button_right = nh.subscribe("/sigma7/sigma0/buttons", 1, &TeleOperation::callback_button_right, this);

    // subscribe current state
    sub_current_pose_left = nh.subscribe("/left/cartesian_motion_controller/current_pose", 1, &TeleOperation::current_pose_callback_left, this);
    sub_current_pose_right = nh.subscribe("/right/cartesian_motion_controller/current_pose", 1, &TeleOperation::current_pose_callback_right, this);

    sub_current_velocity_left = nh.subscribe("/left/cartesian_motion_controller/current_velocity", 1, &TeleOperation::current_velocity_callback_left, this);
    sub_current_velocity_right = nh.subscribe("/right/cartesian_motion_controller/current_velocity", 1, &TeleOperation::current_velocity_callback_right, this);

    single_point_force_sub = nh.subscribe("/touch_single_force", 1, &TeleOperation::singlePointForceCallback, this);
    newretractor_force_sub = nh.subscribe("/new_retractor", 1, &TeleOperation::newRetractorCallback, this);
    IOState_sub = nh.subscribe("/right/ur_hardware_interface/io_states", 1, &TeleOperation::IOStateCallback, this);
    // dynamic param config
    f = boost::bind(&TeleOperation::configCallback, this, _1, _2);
    server.setCallback(f);

    outVel.open("/home/ur5e/Code/shared_control/data/apf/vel.txt");
    outP_foce.open("/home/ur5e/Code/shared_control/data/apf/pforce.txt");
    outFile.open("/home/ur5e/Code/shared_control/data/apf/prob.txt");
    output_re_vel.open("/home/ur5e/Code/shared_control/data/vel.txt");
    output_angle.open("/home/ur5e/Code/shared_control/data/angle.txt");
    // APF init
    // target_cylinder = new Cylinder(0.6, 0.0, 0.15);
    // p_current = new Point2D(0, 0, 0);

    // prev em markov
    T_Markov(0, 0) = 0.5;
    T_Markov(0, 1) = 0.5;
    T_Markov(0, 2) = 0.0;
    T_Markov(1, 0) = 1 / 6;
    T_Markov(1, 1) = 1 / 2;
    T_Markov(1, 2) = 1 / 3;
    T_Markov(2, 0) = 0.0;
    T_Markov(2, 1) = 1 / 3;
    T_Markov(2, 2) = 2 / 3;
}

void TeleOperation::configCallback(shared_controller::commandConfig &config, uint32_t level)
{
    drp.apf = config.apf;
    drp.kp = config.kp;
    drp.kd = config.kd;
    drp.blend = config.blend;
    drp.scale = config.scale;
    drp.scale1 = config.scale1;
    drp.delta_step = config.delta_step;
    drp.auto_delta = config.auto_delta;
    drp.force_k = config.force_k;
    drp.angle = config.angle;
}

void TeleOperation::current_pose_callback_left(const geometry_msgs::PoseStampedConstPtr &msgs)
{
    current_pose_left.pose = msgs->pose;
    if (first_flag_left == 0)
    {
        first_flag_left = 2;
    }
}

void TeleOperation::current_pose_callback_right(const geometry_msgs::PoseStampedConstPtr &msgs)
{
    current_pose_right.pose = msgs->pose;
    if (first_flag_right == 0)
    {
        first_flag_right = 2;
    }
    // vector<float> euler = QuaternionToEulerAngle(current_pose_right);
    // ROS_INFO("x: %f, y: %f, z: %f", euler[0], euler[1], euler[2]);
}

void TeleOperation::singlePointForceCallback(const std_msgs::Float64MultiArrayConstPtr &last_msg)
{
    for (int i = 0; i < 8; i++)
    {
        retractor_spForce[i] = last_msg->data[i];
    }
    retractor_nForce[0] = retractor_nForce[1] = 0.0;
    retractor_nForce = netForceCalculation(retractor_spForce);
    std_msgs::Float64MultiArray netforce_msgs;
    netforce_msgs.data.push_back(retractor_nForce[0]);
    netforce_msgs.data.push_back(retractor_nForce[1]);
    netforce_pub.publish(netforce_msgs);
}

void TeleOperation::newRetractorCallback(const std_msgs::Float64MultiArrayConstPtr &last_msgs)
{
    float p1 = last_msgs->data[0];
    float p2 = last_msgs->data[1];
    float p3 = last_msgs->data[2];
    float p4 = last_msgs->data[3];
    float net = p1 + p2 + p3 + p4;
    net_newforce[0] = net;

    // Mz
    net_newTorque[0] = (p1 + p3 - p2 - p4) * 7;
    // My
    net_newTorque[1] = (p3 + p4 - p1 - p2) * 10.5;
}

vector<float> TeleOperation::netForceCalculation(vector<float> &singlePointForce)
{
    for (int i = 0; i < 4; i++)
    {
        retractor_nForce[0] += retractor_spForce[i];
        retractor_nForce[1] += retractor_spForce[i + 4];
    }
    return retractor_nForce;
}

// void TeleManipulation::configCallback(sigma_client::PathGenerationConfig &config, uint32_t level)
// {
//   path_config = config.Path_command;
//   // ROS_INFO("here...");
// }

void TeleOperation::callback_left(const geometry_msgs::PoseStampedConstPtr &last_msgs_left)
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

    // collision detection
    leftRetractor_Coordians[0] = last_pose_left.pose.position.x + delta_position[1];
    leftRetractor_Coordians[1] = last_pose_left.pose.position.y - delta_position[0];
    leftRetractor_Coordians[2] = last_pose_left.pose.position.z + delta_position[2];
    // vector<float> dis = calculateRetractorDis(leftRetractor_Coordians, rightRetractor_Coordians);
    // relativeDistance = sqrt(pow(dis[0], 2) + pow(dis[1], 2) + pow(dis[2], 2));

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
        target_pose_left.pose.position.x = 0.45;
        target_pose_left.pose.position.y = 0.0;
        target_pose_left.pose.position.z = 0.4;
        target_pose_left.pose.orientation.x = 0.0;
        target_pose_left.pose.orientation.y = 1.0;
        target_pose_left.pose.orientation.z = 0.0;
        target_pose_left.pose.orientation.w = 0.0;

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

        if (relativeDistance > 0.1)
        {
            target_pose_left.pose.position.x = last_pose_left.pose.position.x + delta_position[1];
            target_pose_left.pose.position.y = last_pose_left.pose.position.y - delta_position[0];
            target_pose_left.pose.position.z = last_pose_left.pose.position.z + delta_position[2];

            target_pose_left.pose.orientation.x = ur_q_target_left.x();
            target_pose_left.pose.orientation.y = ur_q_target_left.y();
            target_pose_left.pose.orientation.z = ur_q_target_left.z();
            target_pose_left.pose.orientation.w = ur_q_target_left.w();
        }
        else
        {
            target_pose_left.pose = last_pose_left.pose;
        }

        target_pub_left.publish(target_pose_left);

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

void TeleOperation::callback_right(const geometry_msgs::PoseStampedConstPtr &last_msgs_right)
{
    vector<float> cur_vel(3, 0.0);

    q_transform_right.x() = 0.0;
    q_transform_right.y() = 0.0;
    q_transform_right.z() = 0.7071068;
    q_transform_right.w() = 0.7071068;

    // for coordinate test
    // q_transform_right.x() = 0.0;
    // q_transform_right.y() = 0.0;
    // q_transform_right.z() = -0.7071068;
    // q_transform_right.w() = 0.7071068;

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

    // collision detection
    if (drp.blend)
    {
        auto_delta_position = forceToMotionControl(net_newforce[0] / drp.scale1);
        // ROS_INFO("x: %f y: %f z: %f", auto_delta_position[0], auto_delta_position[1], auto_delta_position[2]);
        rightRetractor_Coordians[0] = last_pose_right.pose.position.x + auto_delta_position[0];
        rightRetractor_Coordians[1] = last_pose_right.pose.position.y + auto_delta_position[1];
        rightRetractor_Coordians[2] = last_pose_right.pose.position.z + auto_delta_position[2];
    }
    else
    {
        rightRetractor_Coordians[0] = last_pose_right.pose.position.x - delta_position[4];
        rightRetractor_Coordians[1] = last_pose_right.pose.position.y + delta_position[3];
        rightRetractor_Coordians[2] = last_pose_right.pose.position.z + delta_position[5];
    }

    vector<float> dis = calculateRetractorDis(leftRetractor_Coordians, rightRetractor_Coordians);
    relativeDistance = sqrt(pow(dis[0], 2) + pow(dis[1], 2) + pow(dis[2], 2));
    // ROS_INFO("relative distance: %f", relativeDistance);

    q_cur_right.x() = q_target_right.x();
    q_cur_right.y() = q_target_right.y();
    q_cur_right.z() = q_target_right.z();
    q_cur_right.w() = q_target_right.w();
    q_last_right.x() = last_sigma_right.pose.orientation.x;
    q_last_right.y() = last_sigma_right.pose.orientation.y;
    q_last_right.z() = last_sigma_right.pose.orientation.z;
    q_last_right.w() = last_sigma_right.pose.orientation.w;

    delta_q_right = q_cur_right * q_last_right.conjugate();

    if (first_flag_right == 0 && button_right == 0)
    {

        target_pose_right.pose.position.x = 0.45;
        target_pose_right.pose.position.y = 0.0;
        target_pose_right.pose.position.z = 0.3;
        target_pose_right.pose.orientation.x = 0.0;
        target_pose_right.pose.orientation.y = 1.0;
        target_pose_right.pose.orientation.z = 0.0;
        target_pose_right.pose.orientation.w = 0.0;

        // for coordinate test
        // target_pose_right.pose.orientation.x = 0.0;
        // target_pose_right.pose.orientation.y = -0.258819;
        // target_pose_right.pose.orientation.z = 0.0;
        // target_pose_right.pose.orientation.w = 0.9659258;
        target_pub_right.publish(target_pose_right);

        // calculate APF
        // p_current->x = target_pose_right.pose.position.x;
        // p_current->y = target_pose_right.pose.position.y;
        // p_current->z = target_pose_right.pose.position.z;

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

        // collision threshold
        if (relativeDistance > 0.1)
        {
            if (drp.blend)
            {
                target_pose_right.pose.position.x = last_pose_right.pose.position.x + auto_delta_position[0];
                target_pose_right.pose.position.y = last_pose_right.pose.position.y + auto_delta_position[1];
                target_pose_right.pose.position.z = last_pose_right.pose.position.z + auto_delta_position[2];
                // target_pose_right.pose.position.x = last_pose_right.pose.position.x - delta_position[4];
                // target_pose_right.pose.position.y = last_pose_right.pose.position.y + drp.scale * delta_position[3];
                // target_pose_right.pose.position.z = last_pose_right.pose.position.z + drp.scale * delta_position[5];

                target_pose_right.pose.orientation.x = ur_q_target_right.x();
                target_pose_right.pose.orientation.y = ur_q_target_right.y();
                target_pose_right.pose.orientation.z = ur_q_target_right.z();
                target_pose_right.pose.orientation.w = ur_q_target_right.w();
            }
            else if (drp.apf)
            {
                std_msgs::Float64MultiArray dynamic_param;
                target_pose_right.pose.position.x = last_pose_right.pose.position.x - delta_position[4];
                target_pose_right.pose.position.y = last_pose_right.pose.position.y + delta_position[3];
                target_pose_right.pose.position.z = last_pose_right.pose.position.z + delta_position[5];

                if (apf_flag == 0)
                {
                    p0[0] = target_pose_right.pose.position.x;
                    p0[1] = target_pose_right.pose.position.y;
                    p0[2] = target_pose_right.pose.position.z;
                    apf_flag = 1;
                }
                vel[0] = delta_position[4] / step * 1000;
                vel[1] = delta_position[3] / step * 1000;
                vel[2] = delta_position[5] / step * 1000;

                retractor_cur[0] = (target_pose_right.pose.position.x - p0[0]) * 1000;
                retractor_cur[1] = (target_pose_right.pose.position.y - p0[1]) * 1000;
                retractor_cur[2] = (target_pose_right.pose.position.z - p0[2]) * 1000;
                float force_y = Fedge(retractor_cur, vel, 0);
                float force_z = Fedge(retractor_cur, vel, 1);
                outVel << vel[0] << "\t" << vel[1] << "\t" << vel[2] << endl;
                outP_foce << retractor_cur[0] << "\t" << retractor_cur[1] << "\t" << retractor_cur[2] << "\t" << force_y << "\t" << force_z << "\t" << endl;
                // ROS_INFO("retractor_cur1:%f, retractor_cur2:%f, vel1:%f, vel:%f", retractor_cur[1], retractor_cur[2], vel[1], vel[2]);
                // ROS_INFO("y:%f, z:%f", p0[1], p0[2]);

                vector<float> compensation = forceToMotionControl(force_z);
                forceFeedback(force_z);
                dynamic_param.data.push_back(1);
                dynamicParam_pub.publish(dynamic_param);
                target_pose_right.pose.position.z -= compensation[0];

                target_pose_right.pose.orientation.x = ur_q_target_right.x();
                target_pose_right.pose.orientation.y = ur_q_target_right.y();
                target_pose_right.pose.orientation.z = ur_q_target_right.z();
                target_pose_right.pose.orientation.w = ur_q_target_right.w();

                delta_last[0] = delta_position[4];
                delta_last[1] = delta_position[3];
                delta_last[2] = delta_position[5];
            }
            else
            {
                float tmp_x = delta_position[3] * sin(drp.angle * PI / 180) + delta_position[4] * cos(drp.angle * PI / 180);
                float tmp_y = delta_position[3] * cos(drp.angle * PI / 180) - delta_position[4] * sin(drp.angle * PI / 180);

                target_pose_right.pose.position.x = last_pose_right.pose.position.x - tmp_x;
                target_pose_right.pose.position.y = last_pose_right.pose.position.y + tmp_y;
                target_pose_right.pose.position.z = last_pose_right.pose.position.z + delta_position[5];

                target_pose_right.pose.orientation.x = ur_q_target_right.x();
                target_pose_right.pose.orientation.y = ur_q_target_right.y();
                target_pose_right.pose.orientation.z = ur_q_target_right.z();
                target_pose_right.pose.orientation.w = ur_q_target_right.w();
            }
            // vector<float> euler = QuaternionToEulerAngle(last_pose_right);
            // ROS_INFO("x: %f, y: %f, z: %f", euler[0], euler[1], euler[2]);
        }
        else
        {
            target_pose_right.pose = last_pose_right.pose;
        }

        target_pub_right.publish(target_pose_right);
        last_pose_right.pose = target_pose_right.pose;
    }
    else
    {

        last_pose_right.header.stamp = ros::Time::now();
        last_pose_right.header.frame_id = "right_base_link";
        target_pub_right.publish(last_pose_right);

        // p_current->x = last_pose_right.pose.position.x;
        // p_current->y = last_pose_right.pose.position.y;
        // p_current->z = last_pose_right.pose.position.z;
    }

    last_sigma_right.pose.position = last_msgs_right->pose.position;
    last_sigma_right.pose.orientation.x = q_target_right.x();
    last_sigma_right.pose.orientation.y = q_target_right.y();
    last_sigma_right.pose.orientation.z = q_target_right.z();
    last_sigma_right.pose.orientation.w = q_target_right.w();

    // vf.eta_p = sc.drp.eta_p;
    // vf.eta_v = sc.drp.eta_v;
    // target_cylinder->R = sc.drp.radius;
    // vf.PublishVirtualForce(*p_current, *target_cylinder, cur_vel);
}

void TeleOperation::callback_vel_left(const geometry_msgs::TwistStampedConstPtr &last_vel)
{
}

void TeleOperation::callback_vel_right(const geometry_msgs::TwistStampedConstPtr &last_vel)
{
    current_twist_right.twist.linear.x = last_vel->twist.linear.x;
    current_twist_right.twist.linear.y = last_vel->twist.linear.y;
    current_twist_right.twist.linear.z = last_vel->twist.linear.z;
    velocity_right[0] = current_twist_right.twist.linear.x;
    velocity_right[1] = current_twist_right.twist.linear.y;
    velocity_right[2] = current_twist_right.twist.linear.z;
    if (button_right == 1)
    {

        if (abs(velocity_right[0]) > EPSILON || abs(velocity_right[0]) > EPSILON || abs(velocity_right[0]) > EPSILON)
        {
            float total = sqrt(pow(velocity_right[0], 2) + pow(velocity_right[1], 2) + pow(velocity_right[2], 2));
            angle_right[0] = acos(velocity_right[0] / total) * 180 / PI;
            angle_right[1] = acos(velocity_right[1] / total) * 180 / PI;
            angle_right[2] = acos(velocity_right[2] / total) * 180 / PI;
            ROS_INFO("x:%f y:%f,z:%f", angle_right[0], angle_right[1], angle_right[2]);
            // output_angle << angle_right[0] << "\t" << angle_right[1] << "\t" << angle_right[2] << endl;
        }
    }
}

float TeleOperation::Fedge(vector<float> &retractor_cur, vector<float> &vel, int id)
{
    float Fedge = 0.0;
    float d = 0.0;
    float v = 0.0;
    // y
    if (id == 0)
    {
        d = retractor_cur[1];
        v = vel[1];
    }
    else
    {
        d = retractor_cur[2];
        v = vel[2];
    }
    if (-(edge_width - retractor_width / 2) <= d && d < 0.0)
    {
        Fedge = eta_rep * pow(d, 2) / 3;
    }
    else if (d >= 0.0 && d <= (edge_width - retractor_width / 2))
    {
        Fedge = -eta_rep * pow(d, 2) / 3;
    }
    else if (-edge_width < d && d < -(edge_width - retractor_width / 2))
    {
        Fedge = eta_rep * v * pow(e, -d);
        Fedge = 0;
    }
    else if ((edge_width - retractor_width / 2) < d && d < edge_width)
    {
        Fedge = eta_rep * v * pow(e, d);
        Fedge = 0;
    }

    return Fedge;
}

vector<float> TeleOperation::forceToMotionControl(float retractor_nForce)
{
    float dis = drp.kp * (retractor_nForce - 4.0) * drp.delta_step + drp.kd * (retractor_nForce - last_retractor_force) * step;

    vector<float> delta_position_tmp(3, 0.0);
    // delta_position_tmp[0] = dis * sin(30 * PI / 180);
    // delta_position_tmp[2] = -dis * cos(30 * PI / 180);
    // ROS_INFO("dis: %f, x: %f, y: %f, z: %f", dis, delta_position_tmp[0], delta_position_tmp[1], delta_position_tmp[2]);
    delta_position_tmp[0] = dis;
    last_retractor_force = retractor_nForce;
    return delta_position_tmp;
}

void TeleOperation::forceFeedback(float fz)
{
    geometry_msgs::WrenchStamped virtual_force;

    virtual_force.header.frame_id = "base_link";
    virtual_force.header.stamp = ros::Time::now();

    virtual_force.wrench.force.x = 0.0;
    virtual_force.wrench.force.y = 0.0;
    virtual_force.wrench.force.z = fz * drp.force_k;
    virtual_force.wrench.torque.x = 0.0;
    virtual_force.wrench.torque.y = 0.0;
    virtual_force.wrench.torque.z = 0.0;
    vfForce_pub.publish(virtual_force);
}

vector<float> TeleOperation::calculateRetractorDis(vector<float> &leftRetractor, vector<float> &rightRetractor)
{
    // 输入已知的坐标
    vector<float> left_base(3, 0.0f);
    vector<float> right_base(3, 0.0f);
    left_base[0] = -0.7;
    left_base[1] = 0.0;
    left_base[2] = 0.0;
    right_base[0] = 0.7;
    right_base[1] = 0.0;
    right_base[2] = 0.0;

    vector<float> relative_distance(3, 0.0f);
    float tmp_left = leftRetractor[0] + left_base[0];
    float tmp_right = -rightRetractor[0] + right_base[0];
    relative_distance[0] = abs(tmp_right - tmp_left);
    relative_distance[1] = abs(rightRetractor[1] - leftRetractor[1]);
    relative_distance[2] = abs(rightRetractor[2] - leftRetractor[2]);

    return relative_distance;
}

void TeleOperation::callback_button_left(const sensor_msgs::JoyConstPtr &last_button_left)
{
    button_left = last_button_left->buttons[0];
}

void TeleOperation::callback_button_right(const sensor_msgs::JoyConstPtr &last_button_right)
{
    button_right = last_button_right->buttons[0];
}

void TeleOperation::current_velocity_callback_left(const geometry_msgs::TwistStampedConstPtr &last_velocity)
{
    velocity_left[0] = last_velocity->twist.linear.x * cos(120 * PI / 180);
    +last_velocity->twist.linear.z *cos(210 * PI / 180);
    velocity_left[1] = last_velocity->twist.linear.y;
    velocity_left[2] = last_velocity->twist.linear.x * cos(30 * PI / 180) + last_velocity->twist.linear.z * cos(120 * PI / 180);
}

void TeleOperation::current_velocity_callback_right(const geometry_msgs::TwistStampedConstPtr &last_velocity)
{
    velocity_right[0] = last_velocity->twist.linear.x;
    velocity_right[1] = last_velocity->twist.linear.y;
    velocity_right[2] = last_velocity->twist.linear.z;
    if (abs(last_velocity->twist.linear.x) > EPSILON && abs(last_velocity->twist.linear.y) > EPSILON && abs(last_velocity->twist.linear.z) > EPSILON)
    {
        velocity_right[0] = last_velocity->twist.linear.x * cos(120 * PI / 180);
        +last_velocity->twist.linear.z *cos(210 * PI / 180);
        velocity_right[1] = last_velocity->twist.linear.y;
        velocity_right[2] = last_velocity->twist.linear.x * cos(30 * PI / 180) + last_velocity->twist.linear.z * cos(120 * PI / 180);

        float rx_cos = velocity_right[0] / (sqrt(pow(velocity_right[0], 2) + pow(velocity_right[1], 2) + pow(velocity_right[2], 2)));
        float ry_cos = velocity_right[1] / (sqrt(pow(velocity_right[0], 2) + pow(velocity_right[1], 2) + pow(velocity_right[2], 2)));
        float rz_cos = velocity_right[2] / (sqrt(pow(velocity_right[0], 2) + pow(velocity_right[1], 2) + pow(velocity_right[2], 2)));
        angle_right[0] = acos(rx_cos) * 180 / PI;
        angle_right[1] = acos(ry_cos) * 180 / PI;
        angle_right[2] = acos(rz_cos) * 180 / PI;
        output_re_vel << velocity_right[0] << "\t" << velocity_right[1] << "\t" << velocity_right[2] << std::endl;
        output_angle << angle_right[0] << "\t";
        output_angle << angle_right[1] << "\t";
        output_angle << angle_right[2] << std::endl;
        //     if (angle_right[2] > 90)
        //     {
        //         angle_right[2] = 90;
        //     }
        //     updateProbability();
        // }
    }
}

void TeleOperation::IOStateCallback(const ur_msgs::IOStatesConstPtr &state)
{
    bool IOstate = state->digital_in_states[0].state;
    if (IOstate)
    {
        iostates = 1;
        ROS_INFO("1111111111111111111111...............");
    }
    else
    {
        iostates = 0;
    }
    // iostates = state->digital_in_states[0];
}
void TeleOperation::updateProbability()
{
    float angle_right_xy = max(angle_right[0], angle_right[1]);
    probability[0][0] = T_Markov(0, 0) * exp(-(90 - angle_right[2]) * lambda) * exp(-velocity_right[2] * beta);
    probability[0][1] = T_Markov(0, 1) * exp(-angle_right[2] * lambda) * exp(-velocity_right[2] * beta);
    probability[0][2] = 0.0;
    probability[1][0] = T_Markov(1, 0) * exp(-(90 - angle_right[2]) * lambda);
    probability[1][1] = T_Markov(1, 1) * exp(-angle_right[2] * lambda) * exp(-velocity_right[2] * beta);
    probability[1][2] = T_Markov(1, 2) * exp(-abs(1.0 - retractor_nForce[1])) * exp(-angle_right[2] * lambda) * exp(-velocity_right[2] * beta);
    probability[2][0] = 0.0;
    probability[2][1] = T_Markov(2, 1) * exp(-abs(1.0 - retractor_nForce[1])) * exp(-angle_right[2] * lambda);
    probability[2][2] = T_Markov(2, 2) * exp(-abs(1.0 - retractor_nForce[1])) * exp(-angle_right[2] * lambda);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (j == 2)
            {
                outFile << probability[i][j] << std::endl;
            }
            else
            {
                outFile << probability[i][j] << "\t";
            }
        }
    }
}

// Eigen::Quaterniond TeleOperation::scaleRotation(Eigen::Quaterniond &q_current, double scale)
// {
//     Eigen::Quaterniond scale_q;
//     std::vector<double> eulerAngles = toEulerAngle(q_current);
//     for (int i = 0; i < 3; i++)
//     {
//         eulerAngles[0] *= scale;
//         eulerAngles[1] *= scale;
//         eulerAngles[2] *= scale;
//     }
//     Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngles[0], ::Eigen::Vector3d::UnitX()));
//     Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngles[1], ::Eigen::Vector3d::UnitY()));
//     Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngles[2], ::Eigen::Vector3d::UnitZ()));
//     scale_q = rollAngle * pitchAngle * yawAngle;

//     return scale_q;
// }

vector<float> TeleOperation::QuaternionToEulerAngle(geometry_msgs::PoseStamped &pose)
{

    Eigen::Quaterniond q;
    q.x() = pose.pose.orientation.x;
    q.y() = pose.pose.orientation.y;
    q.z() = pose.pose.orientation.z;
    q.w() = pose.pose.orientation.w;

    vector<float> eulerAngles(3);
    // roll (x-axis rotation)
    float sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    float cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    eulerAngles[0] = atan2(sinr_cosp, cosr_cosp) * 180 / PI;

    // pitch (y-axis rotation)
    float sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        eulerAngles[1] = copysign(M_PI / 2, sinp) * 180 / PI; // use 90 degrees if out of range
    else
        eulerAngles[1] = asin(sinp) * 180 / PI;

    // yaw (z-axis rotation)
    float siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    float cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    eulerAngles[2] = atan2(siny_cosp, cosy_cosp) * 180 / PI;

    return eulerAngles;
}

TouchTeleOperation::TouchTeleOperation()
{

    // 发布机器人坐标
    target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_motion_controller/target_frame", 1);
    // 订阅，touch位置以及启动按钮
    omni_pose_sub = nh.subscribe("/phantom/pose", 1, &TouchTeleOperation::PoseCallback, this);
    button = nh.subscribe("/phantom/button", 1, &TouchTeleOperation::buttonCallback, this);
}

void TouchTeleOperation::PoseCallback(const geometry_msgs::PoseStampedConstPtr &last_msgs)
{

    // xlh:获取touch目前姿态
    q_omni.x() = last_msgs->pose.orientation.x;
    q_omni.y() = last_msgs->pose.orientation.y;
    q_omni.z() = last_msgs->pose.orientation.z;
    q_omni.w() = last_msgs->pose.orientation.w;

    // xlh:进行转换
    // q_target = q_transform * q_omni;
    q_target = q_omni;

    // 创建目标
    geometry_msgs::PoseStamped target_pose;

    // 设置时间戳
    target_pose.header.stamp = ros::Time::now();
    // 设置坐标
    target_pose.header.frame_id = "base_link";

    // 计算出touch位置相对变换
    delta_position_touch[0] = last_msgs->pose.position.x - last_omni.pose.position.x;
    delta_position_touch[1] = last_msgs->pose.position.y - last_omni.pose.position.y;
    delta_position_touch[2] = last_msgs->pose.position.z - last_omni.pose.position.z;
    // 左乘旋转矩阵，转换坐标系
    // delta_position[0] = (last_msgs->pose.position.x - last_omni.pose.position.x) * sqrt(2) / 2 +
    //                     (last_msgs->pose.position.y - last_omni.pose.position.y) * sqrt(2) / 2;
    // delta_position[1] = -(last_msgs->pose.position.x - last_omni.pose.position.x) * sqrt(2) / 2 +
    //                     (last_msgs->pose.position.y - last_omni.pose.position.y) * sqrt(2) / 2;
    // delta_position[2] = last_msgs->pose.position.z - last_omni.pose.position.z;

    //  转换好的touch姿态设为本帧
    q_cur.x() = q_target.x();
    q_cur.y() = q_target.y();
    q_cur.z() = q_target.z();
    q_cur.w() = q_target.w();

    // 上帧姿态
    q_last.x() = last_omni.pose.orientation.x;
    q_last.y() = last_omni.pose.orientation.y;
    q_last.z() = last_omni.pose.orientation.z;
    q_last.w() = last_omni.pose.orientation.w;

    // 注意，先前此处乘反了
    // 计算出touch姿态相对变化
    delta_q = q_last.conjugate() * q_cur;
    Eigen::Vector3d euler = delta_q.toRotationMatrix().eulerAngles(2, 1, 0);
    // 将绕x和z的旋转方向更改
    euler[2] = -euler[2];
    euler[0] = -euler[0];
    Eigen::AngleAxisd rollAngle(euler[0], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(euler[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(euler[2], Eigen::Vector3d::UnitX());
    delta_q = rollAngle * yawAngle * pitchAngle;

    // tf2::Quaternion qtn;
    // qtn.setRPY(euler[2],euler[1] ,euler[0]);
    // delta_q.x = qtn.getX();
    // delta_q.y = qtn.getY();
    // delta_q.z = qtn.getZ();
    // delta_q.w = qtn.getW();

    // delta_q = q_transform * delta_q;

    // 初始化
    if (first_flag == 0 && grey_button == 0)
    {

        // 机械臂初始位姿
        target_pose.pose.position.x = 0.50;
        target_pose.pose.position.y = 0.1;
        target_pose.pose.position.z = 0.25;

        // 现将坐标转换为前y，右x，上z
        // target_pose.pose.position.x = target_pose.pose.position.x * sqrt(2) / 2 +
        //                               target_pose.pose.position.y * sqrt(2) / 2;
        // target_pose.pose.position.y = -target_pose.pose.position.x * sqrt(2) / 2 +
        //                               target_pose.pose.position.y * sqrt(2) / 2;

        // 先绕y转180
        q_transform.x() = 0.0;
        q_transform.y() = 1.0;
        q_transform.z() = 0.0;
        q_transform.w() = 0.0;

        // 再绕z转135                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       （先转后的坐标系）
        q_transform1.x() = 0.0;
        q_transform1.y() = 0.0;
        q_transform1.z() = 0.924;
        q_transform1.w() = 0.383;

        // q_transform = q_transform * q_transform1;

        target_pose.pose.orientation.x = q_transform.x();
        target_pose.pose.orientation.y = q_transform.y();
        target_pose.pose.orientation.z = q_transform.z();
        target_pose.pose.orientation.w = q_transform.w();

        target_pose_pub.publish(target_pose);
        last_pose.pose = target_pose.pose;
        first_flag = 1;

        // 开始
    }
    else if (first_flag == 1 && grey_button == 1)
    {

        // 获取机械臂当前姿态
        ur_q_cur.x() = last_pose.pose.orientation.x;
        ur_q_cur.y() = last_pose.pose.orientation.y;
        ur_q_cur.z() = last_pose.pose.orientation.z;
        ur_q_cur.w() = last_pose.pose.orientation.w;

        // 计算出机械臂目标姿态
        // 感觉此处也乘反了
        ur_q_target = ur_q_cur * delta_q;
        // ur_q_target = delta_q * ur_q_cur;

        // 机械臂目标位置
        target_pose.pose.position.x = last_pose.pose.position.x + delta_position_touch[0];
        target_pose.pose.position.y = last_pose.pose.position.y + delta_position_touch[1];
        target_pose.pose.position.z = last_pose.pose.position.z + delta_position_touch[2];
        // target_pose.pose.orientation.x = last_msgs->pose.orientation.z;
        // target_pose.pose.orientation.y = last_msgs->pose.orientation.w;
        // target_pose.pose.orientation.z = -last_msgs->pose.orientation.x;
        // target_pose.pose.orientation.w = -last_msgs->pose.orientation.y;
        target_pose.pose.orientation.x = ur_q_target.x();
        target_pose.pose.orientation.y = ur_q_target.y();
        target_pose.pose.orientation.z = ur_q_target.z();
        target_pose.pose.orientation.w = ur_q_target.w();

        target_pose_pub.publish(target_pose);
        last_pose.pose = target_pose.pose;

        if (white_button)
        {
            predictor_tmp.updatePolicy();
            // predictor_tmp.updateDistribution();
        }

        // 保持不动
    }
    else
    {

        last_pose.header.stamp = ros::Time::now();
        last_pose.header.frame_id = "base_link";
        target_pose_pub.publish(last_pose);
    }

    last_omni.pose.position = last_msgs->pose.position;
    last_omni.pose.orientation.x = q_target.x();
    last_omni.pose.orientation.y = q_target.y();
    last_omni.pose.orientation.z = q_target.z();
    last_omni.pose.orientation.w = q_target.w();
}

// 是否按下
void TouchTeleOperation::buttonCallback(const omni_msgs::OmniButtonEventConstPtr &button_state)
{
    grey_button = button_state->grey_button;
    white_button = button_state->white_button;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "shared_controller");

    // SharedController sc;
    TeleOperation to;

    // TouchTeleOperation tto;
    // ros::Rate loop_rate(100);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    // while (ros::ok())
    // {
    //     sc.publishFtSensorForce();
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    return 0;
}