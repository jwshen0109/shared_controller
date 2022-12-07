#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "omni_msgs/OmniFeedback.h"
#include <vector>

using namespace std;

ros::Subscriber left_pose_sub;
ros::Subscriber right_pose_sub;
ros::Publisher left_omni_pub;
ros::Publisher right_omni_pub;

void leftPoseCallback(const geometry_msgs::PoseStampedConstPtr& left_msgs){
    ROS_INFO("111");
}

void rightPoseCallback(const geometry_msgs::PoseStampedConstPtr& right_msgs){

}

vector<double> calculateAngle(vector<double> &quaternions){

}

int main(int argc, char **argv){

    ros::init(argc, argv,"feedback");
    ros::NodeHandle nh;
    
    left_pose_sub = nh.subscribe("/left/cartesian_motion_controller/target_frame", 1, &leftPoseCallback);
    right_pose_sub = nh.subscribe("/right/cartesian_motion_controller/target_frame", 1, &rightPoseCallback);
    left_omni_pub = nh.advertise<omni_msgs::OmniFeedback>("/phantom/force_feedback", 1);
    right_omni_pub = nh.advertise<omni_msgs::OmniFeedback>("/phantom/force_feedback", 1);

    while(ros::ok()){

        ros::spinOnce();
    }

    return 0;
}