#include "ur5e_test/force_rotation.h"

force_rotation::force_rotation(ros::NodeHandle nh)
{
    single_point_force_sub = nh.subscribe("/touch_single_force", 1, &force_rotation::singlePointForceCallback, this);
    newretractor_force_sub = nh.subscribe("/touch_new_retractor", 1, &force_rotation::newRetractorCallback, this);
    left_pose_sub = nh.subscribe("/left/cartesian_force_controller/current_position", 1, &force_rotation::leftPoseCallback, this);
    right_pose_sub = nh.subscribe("/right/cartesian_force_controller/current_position", 1, &force_rotation::rightPoseCallback, this);
    final_euler_sub = nh.subscribe("/final_euler", 1, &force_rotation::final_eulerCallback, this);

    ref_force_pub_left = nh.advertise<geometry_msgs::WrenchStamped>("/left/cartesian_force_controller/target_wrench", 1);
    ref_force_pub_right = nh.advertise<geometry_msgs::WrenchStamped>("/right/cartesian_force_controller/target_wrench", 1);
    left_force_pub = nh.advertise<geometry_msgs::WrenchStamped>("/left/cartesian_force_controller/ft_sensor_wrench", 1);
    right_force_pub = nh.advertise<geometry_msgs::WrenchStamped>("/right/cartesian_force_controller/ft_sensor_wrench", 1);
    net_force_pub = nh.advertise<std_msgs::Float64MultiArray>("/touch_netforce", 1);
    delta_distance_pub = nh.advertise<std_msgs::Float64MultiArray>("/delta_distance", 1);
    force_location_pub = nh.advertise<std_msgs::Float64MultiArray>("/force_location", 1);

    f = boost::bind(&force_rotation::configCallback, this, _1, _2);
    server.setCallback(f);
}

void force_rotation::singlePointForceCallback(const std_msgs::Float64MultiArrayConstPtr &last_msg)
{
    for (int i = 0; i < 8; i++)
    {
        singlePointForce[i] = last_msg->data[i];
        if (i > 3 && i <= 6)
        {
            output_singlePoint << singlePointForce[i] << "\t";
        }
        if (i == 7)
        {
            output_singlePoint << singlePointForce[i] << endl;
        }
    }
    net_force[0] = net_force[1] = 0;
    net_force = netForceCalculation(singlePointForce);
    output_right << net_force[0] << "\t";
    output_right << net_force[1] << endl;
    std_msgs::Float64MultiArray netforce_msgs;
    netforce_msgs.data.push_back(net_force[0]);
    netforce_msgs.data.push_back(net_force[1]);
    if (net_force[1] >= 0.05 && bk.rotation_flag == 0)
    {
        if (bk.distance_flag == 0)
        {
            last_distance = rightPose[0];
            bk.distance_flag = 1;
        }
        double res = (rightPose[0] - last_distance) * 1000;
        net_force_xvals.push_back(net_force[1]);
        force_dis_yvals.push_back(res);
        if (net_force[1] >= 0.98)
        {
            singleForce[0] = singlePointForce[7];
            singleForce[1] = singlePointForce[4];
            singleForce[2] = singlePointForce[6];
            singleForce[3] = singlePointForce[5];
        }
    }
    net_force_pub.publish(netforce_msgs);
}

void force_rotation::newRetractorCallback(const std_msgs::Float64MultiArrayConstPtr &last_msgs)
{
    float p1 = last_msgs->data[0];
    float p2 = last_msgs->data[0];
    float p3 = last_msgs->data[0];
    float p4 = last_msgs->data[0];
    float net = p1 + p2 + p3 + p4;
    net_newforce[0] = net;
}

void force_rotation::final_eulerCallback(const std_msgs::Float64MultiArrayConstPtr &last_msg)
{
    bk.calculation_rotation_flag = 1;
    for (int i = 0; i < 3; i++)
    {
        final_euler[i] = last_msg->data[i];
    }
}

vector<float> force_rotation::netForceCalculation(vector<float> &singlePointForce)
{
    for (int i = 0; i < 4; i++)
    {
        net_force[0] += singlePointForce[i];
        net_force[1] += singlePointForce[i + 4];
    }
    return net_force;
}

void force_rotation::ftSensorForcePub(int id)
{

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
    right_sensor_force.wrench.force.x = net_newforce[0];
    right_sensor_force.wrench.force.y = 0.0;
    right_sensor_force.wrench.force.z = 0.0;
    right_sensor_force.wrench.torque.x = 0.0;
    right_sensor_force.wrench.torque.y = 0.0;
    right_sensor_force.wrench.torque.z = 0.0;

    vector<double> torque(4, 0);
    calculateTorque(torque);

    output_torque << torque[0] << "\t";
    output_torque << torque[1] << endl;
    if (config_param.torque)
    {

        if (abs(torque[0]) >= 0.3)
        {
            right_sensor_force.wrench.torque.z = torque[0];
        }
        if (abs(torque[1] >= 0.3))
        {
            right_sensor_force.wrench.torque.y = -torque[1] / 2;
            // ROS_INFO("torque1:%f", torque[1]);
        }
        if (abs(torque[2]) >= 0.3)
        {
            left_sensor_force.wrench.torque.z = torque[2];
        }
        if (abs(torque[3] >= 0.3))
        {
            left_sensor_force.wrench.torque.y = -torque[3] / 2;
            // ROS_INFO("torque1:%f", torque[1]);
        }
        bk.back_flag = 0;
        bk.rotation_finish = 0;
    }
    else
    {
        if (torque[1] >= 5.0)
        {
            bk.occur = true;
        }
        if (bk.rotation_finish == 1)
        {
            if (abs(torque[0]) >= 0.3)
            {
                right_sensor_force.wrench.torque.z = torque[0];
            }
            if (abs(torque[1] >= 0.3))
            {
                right_sensor_force.wrench.torque.y = -torque[1] / 2;
                // ROS_INFO("torque1:%f", torque[1]);
            }
        }
    }

    // ROS_INFO("Mx:%f",torque[0]);
    // ROS_INFO("My:%f",torque[1]);
    // if(torque[0]>=0.4&&torque[1]>=0.4){

    // }

    if (id == 0)
        left_force_pub.publish(left_sensor_force);
    if (id == 1)
        right_force_pub.publish(right_sensor_force);
}

void force_rotation::leftRefForcePub()
{

    geometry_msgs::WrenchStamped ref_force_left;

    ref_force_left.header.stamp = ros::Time::now();
    ref_force_left.header.frame_id = "retractor";
    ref_force_left.wrench.force.x = -config_param.left_force_config;
    ref_force_left.wrench.force.y = 0.0;
    ref_force_left.wrench.force.z = 0.0;
    ref_force_left.wrench.torque.x = 0.0;
    ref_force_left.wrench.torque.y = 0.0;
    ref_force_left.wrench.torque.z = 0.0;

    ref_force_pub_left.publish(ref_force_left);
}

void force_rotation::rightRefForcePub()
{

    geometry_msgs::WrenchStamped ref_force_right;

    ref_force_right.header.stamp = ros::Time::now();
    ref_force_right.header.frame_id = "retractor";
    ref_force_right.wrench.force.x = -config_param.right_force_config;
    ref_force_right.wrench.force.y = 0.0;
    ref_force_right.wrench.force.z = 0.0;
    ref_force_right.wrench.torque.x = 0.0;
    ref_force_right.wrench.torque.y = 0.0;
    ref_force_right.wrench.torque.z = 0.0;

    if (config_param.tongue)
    {
        if (bk.tongue_count <= 100)
        {
            ref_force_right.wrench.force.x = 0.0;
        }
        bk.tongue_count++;
    }
    if (config_param.tongue == false)
    {
        bk.tongue_count = 0;
    }

    // rotation function
    if (config_param.right_rotation)
    {
        // if(bk.occur){
        if (net_force[1] >= 0.98 && bk.back_flag == 0)
        {
            bk.back_flag = 1;
        }
        if (bk.back_flag == 1 && bk.rotation_finish == 0)
        {
            ref_force_right.wrench.force.x = 0.0;
            // if(bk.calculation_polyfit==0){
            //     calculationForceDistance();
            // }
            // if(bk.calculation_rotation_flag==0){
            //     ROS_INFO_STREAM_ONCE("backing...........");
            // }else{
            ROS_INFO_STREAM_ONCE("stop backing....");
            if (bk.back_count <= 300)
            {
                bk.back_count++;
            }
            else
            {
                bk.rotation_flag = 1;
            }
            //}
        }

        if (bk.rotation_flag == 1 && bk.rotation_count <= abs(config_param.right_angle_My) * 35)
        {
            if (bk.rotation_count <= abs(config_param.right_angle_My) * 27)
            {
                if (config_param.right_angle_My > 0)
                {
                    ref_force_right.wrench.torque.y = -1.0;
                }
                else
                {
                    ref_force_right.wrench.torque.y = 1.0;
                }
                ROS_INFO_STREAM_ONCE("rotation start...........");
                if (bk.rotation_count >= abs(config_param.right_angle_My) * 12 && bk.rotation_count <= abs(config_param.right_angle_My) * 16)
                {
                    ref_force_right.wrench.force.x = -0.2;
                }
            }

            if (bk.mx_count <= abs(config_param.right_angle_Mx) * 27)
            {
                if (config_param.right_angle_Mx > 0)
                {
                    ref_force_right.wrench.torque.z = 1.0;
                }
                else
                {
                    ref_force_right.wrench.torque.z = -1.0;
                }
                ROS_INFO_STREAM_ONCE("x-axis rotation start...........");
            }
            // if(bk.mz_count<=abs(config_param.right_angle_Mz)*27){
            //     // if(config_param.right_angle_Mz>0){
            //     //     ref_force_right.wrench.torque.x = -2.0;
            //     // }else{
            //     //     ref_force_right.wrench.torque.x = 2.0;
            //     // }
            //     // ROS_INFO_STREAM_ONCE("z-axis rotation start...........");

            // }
            if (bk.rotation_count >= abs(config_param.right_angle_My) * 30)
            {
                ROS_INFO_STREAM_ONCE("stop y-axis rotation................");
                bk.rotation_finish = 1;
                // bk.first = true;
                // bk.occur = false;
            }
            bk.mx_count++;
            // bk.mz_count++;
            bk.rotation_count++;
        }
    }

    // if(bk.first){
    //     if(net_force[1]>=0.98&&bk.back_flag_sec==0){
    //         bk.back_flag_sec = 1;
    //     }
    //     if(bk.back_flag_sec==1&&bk.rotation_finish==0){
    //         ref_force_right.wrench.force.x = 0.0;
    //         // if(bk.calculation_polyfit==0){
    //         //     calculationForceDistance();
    //         // }
    //         // if(bk.calculation_rotation_flag==0){
    //         //     ROS_INFO_STREAM_ONCE("backing...........");
    //         // }else{
    //             ROS_INFO_STREAM_ONCE("stop backing....");
    //             if(bk.back_count_sec<=300){
    //                 bk.back_count_sec++;
    //             }else{
    //                 bk.rotation_flag_sec = 1;
    //             }
    //         //}
    //     }

    //     if(bk.rotation_flag_sec==1&&bk.rotation_count_sec<=abs(-38)*35){
    //         if(bk.rotation_count_sec<=abs(-38)*27){
    //             //if(config_param.right_angle_My>0){
    //                 //ref_force_right.wrench.torque.y = -1.0;
    //             //}else{
    //                 ref_force_right.wrench.torque.y = 1.0;
    //             //}
    //             ROS_INFO_STREAM_ONCE("rotation start...........");
    //             if(bk.rotation_count_sec>=abs(-38)*12 && bk.rotation_count_sec<=abs(-38)*20){
    //                 ref_force_right.wrench.force.x = -0.2;
    //             }
    //         }

    //         if(bk.mx_count_sec<=abs(-22)*27){
    //             //if(config_param.right_angle_Mx>0){
    //                 //ref_force_right.wrench.torque.z = 1.0;
    //             //}else{
    //                 ref_force_right.wrench.torque.z = -1.0;
    //             //}
    //             ROS_INFO_STREAM_ONCE("x-axis rotation start...........");
    //         }
    //         // if(bk.mz_count<=abs(config_param.right_angle_Mz)*27){
    //         //     // if(config_param.right_angle_Mz>0){
    //         //     //     ref_force_right.wrench.torque.x = -2.0;
    //         //     // }else{
    //         //     //     ref_force_right.wrench.torque.x = 2.0;
    //         //     // }
    //         //     // ROS_INFO_STREAM_ONCE("z-axis rotation start...........");

    //         // }
    //         if(bk.rotation_count_sec >= abs(-38)*30){
    //             ROS_INFO_STREAM_ONCE("stop y-axis rotation................");
    //             bk.rotation_finish = 1;
    //         }
    //         bk.mx_count_sec++;
    //         //bk.mz_count++;
    //         bk.rotation_count_sec++;
    //     }
    // }

    ref_force_pub_right.publish(ref_force_right);
}

void force_rotation::configCallback(ur5e_test::referenceConfig &config, uint32_t level)
{

    config_param.left_force_config = config.left_force;
    config_param.right_force_config = config.right_force;
    config_param.left_rotation = config.left_rotation;
    config_param.right_rotation = config.right_rotation;
    config_param.left_MX = config.left_MX;
    config_param.left_MY = config.left_MY;
    config_param.right_MX = config.right_MX;
    config_param.right_MY = config.right_MY;

    config_param.left_force_data_recorder = config.left_force_recorder;
    config_param.right_force_data_recorder = config.right_force_recorder;
    config_param.right_angle_My = config.right_angle_My;
    config_param.right_angle_Mx = config.right_angle_Mx;
    config_param.right_angle_Mz = config.right_angle_Mz;

    config_param.ptc = config.ptc;
    config_param.torque = config.torque;
    config_param.tongue = config.tongue;
    // ROS_INFO("111......");
    // refForcePub(left_force_config, right_force_config);
}

void force_rotation::leftPoseCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    leftPose[0] = pose->pose.position.x;
    leftPose[1] = pose->pose.position.y;
    leftPose[2] = pose->pose.position.z;
    if (net_force[0] >= 0.05)
    {
        double tmp = pow(leftPose[0], 2) + pow(leftPose[1], 2) + pow(leftPose[2], 2);
        double pose = pow(tmp, 0.5);
        output_pose << pose << endl;
    }
}

void force_rotation::rightPoseCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    rightPose[0] = pose->pose.position.x;
    rightPose[1] = pose->pose.position.y;
    rightPose[2] = pose->pose.position.z;
    if (net_force[1] >= 0.05)
    {
        double tmp = pow(rightPose[0], 2) + pow(rightPose[1], 2) + pow(rightPose[2], 2);
        double pose = pow(tmp, 0.5);
        output_pose << pose << endl;
    }
    // output_pose << rightPose[0] << "\t";
    // output_pose << rightPose[1] << "\t";
    // output_pose << rightPose[2] << endl;
}

void force_rotation::calculationForceDistance()
{

    if (net_force_xvals.size() != force_dis_yvals.size())
    {
        int m = net_force_xvals.size();
        int n = force_dis_yvals.size();
        if (m > n)
        {
            net_force_xvals.erase(net_force_xvals.begin() + n, net_force_xvals.end());
        }
        else
        {
            force_dis_yvals.erase(force_dis_yvals.begin() + m, force_dis_yvals.end());
        }
    }
    Eigen::VectorXd res = polyfit(net_force_xvals, force_dis_yvals, 4);
    double sum = 0;
    vector<double> tmp(4, 0.0);
    for (int i = 0; i < 4; i++)
    {
        tmp[i] = polyeval(res, singleForce[i]);
        sum += tmp[i];
    }
    ROS_INFO("singleForce 0:%f", singleForce[0]);
    ROS_INFO("singleForce 1:%f", singleForce[1]);
    ROS_INFO("singleForce 2:%f", singleForce[2]);
    ROS_INFO("singleForce 3:%f", singleForce[3]);
    ROS_INFO("sum:%f", sum);
    output_polyfit << res(0) << "\t";
    output_polyfit << res(1) << "\t";
    output_polyfit << res(2) << "\t";

    std_msgs::Float64MultiArray distance_msgs;
    vector<double> delta_distance(4, 0);
    for (int i = 0; i < 4; i++)
    {
        delta_distance[i] = -1 * (tmp[i] - sum / 4);
        if (i != 3)
        {
            output_polyfit << delta_distance[i] << "\t";
        }
        else
        {
            output_polyfit << delta_distance[i] << endl;
        }
        distance_msgs.data.push_back(delta_distance[i]);
    }
    ROS_INFO("delta_distance publish success, include: point 1: %f\t, point 2:%f\t, point 3:%f\t, point 4:%f\n", delta_distance[0], delta_distance[1], delta_distance[2], delta_distance[3]);
    delta_distance_pub.publish(distance_msgs);
    bk.calculation_polyfit = 1;
    ROS_INFO("delta_distance generation success.......");
}

Eigen::VectorXd force_rotation::polyfit(vector<double> &xvals, vector<double> &yvals, int order)
{

    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::VectorXd yvals_tmp = Eigen::VectorXd::Zero(yvals.size());
    for (int i = 0; i < yvals.size(); i++)
    {
        yvals_tmp(i) = yvals[i];
    }
    Eigen::MatrixXd A(xvals.size(), order + 1);
    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }
    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals[j];
        }
    }
    auto Q = A.householderQr();
    auto result = Q.solve(yvals_tmp);
    return result;
}

double force_rotation::polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

void force_rotation::calculateTorque(vector<double> &torque)
{

    if (net_force[1] >= 0.04)
    {
        std_msgs::Float64MultiArray location_msg;
        double delta_x = (singlePointForce[5] + singlePointForce[7] - singlePointForce[4] - singlePointForce[6]) * 5.5 / net_force[1];
        double delta_y = (singlePointForce[4] + singlePointForce[7] - singlePointForce[6] - singlePointForce[5]) * 2.8 / net_force[1];
        location_msg.data.push_back(delta_x);
        location_msg.data.push_back(delta_y);
        output_location << delta_x << "\t";
        output_location << delta_y << endl;
        torque[0] = net_force[1] * delta_y;
        torque[1] = net_force[1] * delta_x;
        force_location_pub.publish(location_msg);
    }
    if (net_force[0] >= 0.04)
    {
        double delta_x_l = (singlePointForce[0] + singlePointForce[2] - singlePointForce[1] - singlePointForce[3]) * 5.5 / net_force[0];
        double delta_y_l = (singlePointForce[0] + singlePointForce[3] - singlePointForce[1] - singlePointForce[2]) * 2.8 / net_force[0];
        torque[2] = net_force[0] * delta_y_l;
        torque[3] = net_force[0] * delta_x_l;
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "force_rotation");
    ros::NodeHandle nh;

    ros::Rate loop_rate(100);

    force_rotation *force_control = new force_rotation(nh);

    // force_control->output_left.open("/home/jwshen-310/Code/RAL/data/left_force.txt");
    // force_control->output_right.open("/home/jwshen-310/Code/RAL/data/right_force.txt");
    // force_control->output_location.open("/home/jwshen-310/Code/RAL/data/location.txt");
    // force_control->output_polyfit.open("/home/jwshen-310/Code/RAL/data/polyfit.txt");
    // force_control->output_singlePoint.open("/home/jwshen-310/Code/RAL/data/singlePoint.txt");
    // force_control->output_torque.open("/home/jwshen-310/Code/RAL/data/torque.txt");
    // force_control->output_pose1.open("/home/jwshen-310/Code/RAL/data/leftpose.txt");
    // force_control->output_pose.open("/home/jwshen-310/Code/RAL/data/rightpose.txt");

    while (ros::ok())
    {
        force_control->ftSensorForcePub(0);
        force_control->ftSensorForcePub(1);
        force_control->leftRefForcePub();
        force_control->rightRefForcePub();
        ros::spinOnce();
        loop_rate.sleep();
    }
}