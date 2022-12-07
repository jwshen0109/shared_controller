#include "sigma7/multiSigmaDevices.h"

//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
MultiSigmaDevices::MultiSigmaDevices(ros::NodeHandle n)
{
    n.param<string>("left_hand", left_hand, "/sigma1");
    n.param<string>("right_hand", right_hand, "/sigma0");
    n.param<int>("id_left", id_left, 1);
    n.param<int>("id_right", id_right, 0);
    // setup the publishers and the subscriber
    pose_pub_left = n.advertise<geometry_msgs::PoseStamped>("/sigma7" + left_hand + "/pose", 1, 0);
    pose_pub_right = n.advertise<geometry_msgs::PoseStamped>("/sigma7" + right_hand + "/pose", 1, 0);
    twist_pub_left = n.advertise<geometry_msgs::TwistStamped>("/sigma7" + left_hand + "/twist", 1, 0);
    twist_pub_right = n.advertise<geometry_msgs::TwistStamped>("/sigma7" + right_hand + "/twist", 1, 0);
    gripper_pub_left = n.advertise<std_msgs::Float32>("/sigma7" + left_hand + "gripper_angle", 1, 0);
    gripper_pub_right = n.advertise<std_msgs::Float32>("/sigma7" + right_hand + "gripper_angle", 1, 0);
    buttons_pub_left = n.advertise<sensor_msgs::Joy>("/sigma7" + left_hand + "/buttons", 1, 0);
    buttons_pub_right = n.advertise<sensor_msgs::Joy>("/sigma7" + right_hand + "/buttons", 1, 0);

    // std::string wrench_topic("/sigma/force_feedback");
    // n.getParam("wrench_topic", wrench_topic);
    // sub_wrench = n.subscribe(wrench_topic, 1, &MultiSigmaDevices::WrenchCallback,
    //                          this);
    wrench_sub_left = n.subscribe("/sigma1/force_feedback", 1, &MultiSigmaDevices::WrenchCallbackLeft, this);
    wrench_sub_right = n.subscribe("/sigma0/force_feedback", 1, &MultiSigmaDevices::WrenchCallbackRight, this);
    // params
    n.param<bool>("enable_gripper_button", enable_gripper_button, 0);
    n.param<bool>("lock_orientation", lock_orient, 0);

    // calibrate the devices
    if (CalibrateDevice(id_left) == -1 || CalibrateDevice(id_right) == -1)
    {
        ros::shutdown();
    }
    buttons_msg_left.buttons.push_back(0);
    buttons_msg_left.buttons.push_back(0);
    buttons_msg_right.buttons.push_back(0);
    buttons_msg_right.buttons.push_back(0);
}

//------------------------------------------------------------------------------
// WrenchCallback
void MultiSigmaDevices::WrenchCallbackLeft(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    // newDataDirect = true;
    wrench_left.wrench = msg->wrench;
}

void MultiSigmaDevices::WrenchCallbackRight(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    // newDataDirect = true;
    wrench_right.wrench = msg->wrench;
}

//------------------------------------------------------------------------------
// CalibrateDevice
int MultiSigmaDevices::CalibrateDevice(int id)
{

    ROS_INFO("Calibrating device %i ...", id);

    // open device
    if (drdOpenID((char)id) < 0)
    {
        ROS_ERROR("No device %i found. dhd says: %s", id, dhdErrorGetLastStr());
        dhdSleep(2.0);
        drdClose((char)id);
        return -1;
    }

    // Calibrate the device if it is not already calibrated;
    if (drdIsInitialized((char)id))
    {
        ROS_INFO("Device %i is already calibrated.", id);
    }
    else if (drdAutoInit((char)id) < 0)
    {
        ROS_ERROR("Initialization of device %i failed. dhd says: (%s)", id,
                  dhdErrorGetLastStr());
        dhdSleep(2.0);
    }

    // // center of workspace
    //	 double nullPose[DHD_MAX_DOF] = { 0.0, 0.0, 0.0, //base  (translations)
    //	                                  0.0, 0.0, 0.0, //wrist (rotations)
    //	                                  0.0 };         //gripper
    // //move to center
    // drdMoveTo (nullPose);

    // stop regulation (and leave force enabled)
    drdStop(true, (char)id);

    // enable force
    dhdEnableForce(DHD_ON, (char)id);

    //    dhdSetGravityCompensation(DHD_ON, (char)id);
    dhdSleep(0.2);
    // Enable the gripper button
    if (enable_gripper_button)
        dhdEmulateButton(DHD_ON, (char)id);

    ROS_INFO("Device %i ready.", id);
    return 0;
}

int MultiSigmaDevices::ReadMeasurementsFromDevice(int id)
{

    // -------------------------------
    // Pose
    double p[3];
    double orient_m[3][3];
    // Reading the data from the device
    dhdGetPositionAndOrientationFrame(&p[0], &p[1], &p[2], orient_m, (char)id);

    // convert to pose message
    KDL::Rotation rot;
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            rot(r, c) = orient_m[r][c];
        }
    }
    if (id == id_left)
    {
        tf::poseKDLToMsg(KDL::Frame(rot, KDL::Vector(p[0], p[1], p[2])), pose_msg_left.pose);
        // stamp the msg
        pose_msg_left.header.stamp = ros::Time::now();

        double v[6];

        dhdGetLinearVelocity(&v[0], &v[1], &v[2], (char)id);
        dhdGetAngularVelocityRad(&v[3], &v[4], &v[5], (char)id);
        // convert to twist message
        twist_msg_left.twist.linear.x = v[0];
        twist_msg_left.twist.linear.y = v[1];
        twist_msg_left.twist.linear.z = v[2];
        twist_msg_left.twist.angular.x = v[3];
        twist_msg_left.twist.angular.y = v[4];
        twist_msg_left.twist.angular.z = v[5];
        // stamp the msg
        twist_msg_left.header.stamp = ros::Time::now();

        // gripper
        double temp;
        dhdGetGripperAngleRad(&temp);
        gripper_angle_left.data = (float)temp;

        for (int i = 0; i < 2; ++i)
        {
            buttons_previous_state_left[i] = buttons_state_left[i];
            buttons_state_left[i] = dhdGetButton(i, (char)id);
        }
    }
    else
    {
        tf::poseKDLToMsg(KDL::Frame(rot, KDL::Vector(p[0], p[1], p[2])), pose_msg_right.pose);
        // stamp the msg
        pose_msg_right.header.stamp = ros::Time::now();

        double v[6];

        dhdGetLinearVelocity(&v[0], &v[1], &v[2], (char)id);
        dhdGetAngularVelocityRad(&v[3], &v[4], &v[5], (char)id);
        // convert to twist message
        twist_msg_right.twist.linear.x = v[0];
        twist_msg_right.twist.linear.y = v[1];
        twist_msg_right.twist.linear.z = v[2];
        twist_msg_right.twist.angular.x = v[3];
        twist_msg_right.twist.angular.y = v[4];
        twist_msg_right.twist.angular.z = v[5];
        // stamp the msg
        twist_msg_right.header.stamp = ros::Time::now();

        // gripper
        double temp;
        dhdGetGripperAngleRad(&temp);
        gripper_angle_right.data = (float)temp;

        for (int i = 0; i < 2; ++i)
        {
            buttons_previous_state_right[i] = buttons_state_right[i];
            buttons_state_right[i] = dhdGetButton(i, (char)id);
        }
    }

    // ------------------------------

    // ------------------------------
    // buttons
    // saving the previous states of gripper button and pedal

    return 0;
}

void MultiSigmaDevices::PublishPoseTwistButtonPedal(int id)
{

    if (id == id_left)
    {
        pose_pub_left.publish(pose_msg_left);
        twist_pub_left.publish(twist_msg_left);
        gripper_pub_left.publish(gripper_angle_left);

        if ((buttons_state_left[0] != buttons_previous_state_left[0]) ||
            (buttons_state_left[1] != buttons_previous_state_left[1]))
        {

            // populate the message
            buttons_msg_left.buttons[0] = buttons_state_left[0];
            buttons_msg_left.buttons[1] = buttons_state_left[1];
            // publish it
            buttons_pub_left.publish(buttons_msg_left);
        }
    }
    else
    {
        pose_pub_right.publish(pose_msg_right);
        twist_pub_right.publish(twist_msg_right);
        gripper_pub_right.publish(gripper_angle_right);

        if ((buttons_state_right[0] != buttons_previous_state_right[0]) ||
            (buttons_state_right[1] != buttons_previous_state_right[1]))
        {

            // populate the message
            buttons_msg_right.buttons[0] = buttons_state_right[0];
            buttons_msg_right.buttons[1] = buttons_state_right[1];
            // publish it
            buttons_pub_right.publish(buttons_msg_right);
        }
    }

    // publish buttons when there is a change
}

void MultiSigmaDevices::HandleWrench(int id)
{

    // should we use new_wrench_msg?
    if (buttons_state_left[1] == 1)
    {
        if (dhdSetForceAndTorqueAndGripperForce(wrench_left.wrench.force.x,
                                                wrench_left.wrench.force.y,
                                                wrench_left.wrench.force.z,
                                                wrench_left.wrench.torque.x,
                                                wrench_left.wrench.torque.y,
                                                wrench_left.wrench.torque.z,
                                                0.0, (char)id) < DHD_NO_ERROR)
        {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        }
        dhdGetOrientationRad(&locked_orient[0], &locked_orient[1], &locked_orient[2]);
    }
    else if (lock_orient)
    {
        drdRegulatePos(false);
        drdRegulateRot(true);
        drdRegulateGrip(false);
        drdStart();
        drdMoveToRot(locked_orient[0], locked_orient[1], locked_orient[2]);
        drdStop(true);
    }
    else
    {
        if (dhdSetForceAndTorqueAndGripperForce(.0, .0, .0, .0, .0, .0, 0.,
                                                (char)id) < DHD_NO_ERROR)
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
    }
}

void MultiSigmaDevices::setLeftForceFeedback()
{
    dhdSetGravityCompensation(DHD_ON, (char)id_left);

    if (buttons_state_left[0] == 0)
    {
        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (char)id_left) < DHD_NO_ERROR)
        {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            ros::shutdown();
        }
    }
    else
    {
        if (dhdSetForceAndTorqueAndGripperForce(wrench_left.wrench.force.x,
                                                wrench_left.wrench.force.y,
                                                wrench_left.wrench.force.z,
                                                wrench_left.wrench.torque.x,
                                                wrench_left.wrench.torque.y,
                                                wrench_left.wrench.torque.z,
                                                0.0, (char)id_left) < DHD_NO_ERROR)
        {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            ros::shutdown();
        }
    }
}

void MultiSigmaDevices::setRightForceFeedback()
{
    dhdSetGravityCompensation(DHD_ON, (char)id_right);

    if (buttons_state_right[0] == 0)
    {
        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (char)id_right) < DHD_NO_ERROR)
        {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            ros::shutdown();
        }
    }
    else
    {
        if (dhdSetForceAndTorqueAndGripperForce(wrench_right.wrench.force.x,
                                                wrench_right.wrench.force.y,
                                                wrench_right.wrench.force.z,
                                                wrench_right.wrench.torque.x,
                                                wrench_right.wrench.torque.y,
                                                wrench_right.wrench.torque.z,
                                                0.0, (char)id_right) < DHD_NO_ERROR)
        {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            ros::shutdown();
        }
    }
}

void CheckAvailableDevices(int &devs)
{

    while (ros::ok() && devs == 0)
    {
        for (int i = 0; i < 2; i++)
        {
            if (drdOpenID((char)i) > -1)
                devs = i + 1;
            ROS_INFO("opend devices: No.%i", devs);
        }
        ros::Rate r(0.5);
        r.sleep();
        ROS_INFO("Looking for connected devices...");
    }

    ROS_INFO("Found %i Device(s)", devs);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sigma7");
    ros::NodeHandle n(ros::this_node::getName());

    // Locking call looking for connected devices.
    // devs is the number of available devices
    int devs = 0;
    CheckAvailableDevices(devs);

    // declare device pointers
    MultiSigmaDevices *sigma = new MultiSigmaDevices(n);
    // Initialize devices
    // for (int i = 0; i < devs; i++ ){
    //     std::stringstream dev_names;
    //     dev_names << "sigma" << i;
    //     sigma[i] = new MultiSigmaDevices(n, dev_names.str());
    // }

    // get the frequency parameter
    double rate;
    n.param<double>("frequency", rate, 1000);
    ROS_INFO("Set frequency: %f", rate);
    ros::Rate loop_rate(rate);

    ROS_INFO("Initialization done.");
    ROS_INFO("devs: %i", devs);

    while (ros::ok())
    {

        // Reading Sigma measurements and setting the force
        for (int i = 0; i < devs; i++)
        {

            sigma->ReadMeasurementsFromDevice(i);

            sigma->PublishPoseTwistButtonPedal(i);

            sigma->setLeftForceFeedback();
            sigma->setRightForceFeedback();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Ending Session...\n");
    for (int i = 0; i < devs; i++)
    {
        if (dhdClose((char)i) < 0)
            ROS_ERROR(" %s\n", dhdErrorGetLastStr());
        else
            ROS_INFO("Closed device %i", i);
        delete sigma;
    }

    return 0;
}
