#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h" 


int main(int argc, char **argv){
    ros::init(argc, argv,"test");
    ros::NodeHandle n;

    //ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("chatter", 1); //定义发布消息的名称及sulv

    ros::Rate loop_rate(500);

    int loop = 0;
    while(ros::ok()) {

        ROS_INFO("loop: %i", loop);
        loop++;
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
    
}