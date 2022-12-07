#ifndef FORWARD_KINEMATIC_H_
#define FORWARD_KINEMATIC_H_

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include <vector>
#include <eigen3/Eigen/Dense>

#define PI = 3.141592653589

struct DH_Parameters{

    double d[6] = {163, 0, 0, 134, 100, 100};
    double a[6] = {0, -425, -392.25, 0, 0, 0};
    double alpha[6] = {M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0};

};

class forward_kinematic{

    public:
        forward_kinematic();

        void initMatrix();

        Eigen::Matrix4d T_Matrix(int index);

        Eigen::Matrix4d calculateTMatrix();

        void jointStateCallback(const sensor_msgs::JointStateConstPtr &state);

        std::vector<float> RotToQuaternions(Eigen::Matrix4d &T);
        
        void posePublish();

    private:

        ros::NodeHandle nh;
        ros::Subscriber joint_sub;
        ros::Publisher cartesian_pub;

        double q_current[6];
        DH_Parameters dh;
        Eigen::Matrix4d T01, T12, T23, T34, T45, T56, T_target;

};

#endif //