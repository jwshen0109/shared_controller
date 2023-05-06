#ifndef FORCE_ROTATION_H_
#define FORCE_ROTATION_H_

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <ur5e_test/referenceConfig.h>

#include <Eigen/Dense>

#include <fstream>
#include <vector>

using namespace std;
using namespace Eigen;

struct config_parameters
{
    double left_force_config = 0.0;
    double right_force_config = 0.0;
    double left_MX = 0.0;
    double left_MY = 0.0;
    double right_MX = 0.0;
    double right_MY = 0.0;
    double left_force_data_recorder = 0.0;
    double right_force_data_recorder = 0.0;
    double right_angle_My = 0.0;
    double right_angle_Mx = 0.0;
    double right_angle_Mz = 0.0;

    bool left_rotation = false;
    bool right_rotation = false;
    bool torque = false;
    bool ptc = false;
    bool tongue = false;
};

struct backtracking
{

    int rotation_flag = 0;
    int mydone = 0;
    int mx_count = 0;
    int mz_count = 0;
    int rotation_count = 0;
    int rotation_finish = 0;

    int back_left = 0;
    int rotation_left = 0;
    int back_flag = 0;
    int back_count = 0;
    int rotation_right = 0;
    int calculation_polyfit = 0;
    int calculation_rotation_flag = 0;
    int distance_flag = 0;

    bool occur = false;
    bool first = false;
    int back_flag_sec = 0;
    int back_count_sec = 0;
    int rotation_count_sec = 0;
    int mx_count_sec = 0;
    int rotation_flag_sec = 0;
    int tongue_count = 0;
};

class force_rotation
{
public:
    force_rotation(ros::NodeHandle nh);
    vector<float> netForceCalculation(vector<float> &singlePointForce);
    void singlePointForceCallback(const std_msgs::Float64MultiArrayConstPtr &last_msg);
    void newRetractorCallback(const std_msgs::Float64MultiArrayConstPtr &last_msgs);
    void final_eulerCallback(const std_msgs::Float64MultiArrayConstPtr &last_msg);
    void leftRefForcePub();
    void rightRefForcePub();
    void configCallback(ur5e_test::referenceConfig &config, uint32_t level);
    void ftSensorForcePub(int id);
    void leftPoseCallback(const geometry_msgs::PoseStampedConstPtr &pose);
    void rightPoseCallback(const geometry_msgs::PoseStampedConstPtr &pose);
    void calculationForceDistance();
    Eigen::VectorXd polyfit(vector<double> &xvals, vector<double> &yvals, int order);
    double polyeval(Eigen::VectorXd coeffs, double x);
    void calculateTorque(vector<double> &torque);

public:
    ofstream output_left;
    ofstream output_right;
    ofstream output_location;
    ofstream output_polyfit;
    ofstream output_singlePoint;
    ofstream output_torque;
    ofstream output_pose;
    ofstream output_pose1;

private:
    ros::Publisher ref_force_pub_left;
    ros::Publisher ref_force_pub_right;
    ros::Publisher left_force_pub;
    ros::Publisher right_force_pub;
    ros::Publisher net_force_pub;
    ros::Publisher delta_distance_pub;
    ros::Publisher force_location_pub;

    ros::Subscriber single_point_force_sub;
    ros::Subscriber newretractor_force_sub;
    ros::Subscriber left_pose_sub;
    ros::Subscriber right_pose_sub;
    ros::Subscriber final_euler_sub;

    // dynamic_reconfigure callback setting;
    dynamic_reconfigure::Server<ur5e_test::referenceConfig> server;
    dynamic_reconfigure::Server<ur5e_test::referenceConfig>::CallbackType f;

    config_parameters config_param;
    backtracking bk;

    vector<float> singlePointForce = vector<float>(8, 0.0);

    vector<double> leftPose = vector<double>(3, 0.0);
    vector<double> rightPose = vector<double>(3, 0.0);
    vector<float> net_force = vector<float>(2, 0.0);
    vector<float> net_newforce = vector<float>(2, 0.0);
    vector<double> singleForce = vector<double>(4, 0.0);
    vector<double> final_euler = vector<double>(3, 0.0);
    vector<double> net_force_xvals;
    vector<double> force_dis_yvals;
    // on-line configuration parameter

    double last_distance = 0.0;
};

// class force_calculation{
//     public:
//         force_calculation(ros::NodeHandle& nh);

//     private:

//         ros::Subscriber left_force_sub;
//         ros::Subscriber right_force_sub;

// };
#endif //