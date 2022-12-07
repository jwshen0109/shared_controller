#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <ur5e_test/referenceConfig.h>


using namespace std;

ros::Publisher left_force_pub;
ros::Publisher right_force_pub;

//output data
ofstream output_left;
ofstream output_right;

//on-line configuration parameter
double left_force_config = 0.0;
double right_force_config = 0.0;
double left_MX = 0.0;
double left_MY = 0.0;
double right_MX = 0.0;
double right_MY = 0.0;
double left_force_data_recorder = 0.0;
double right_force_data_recorder = 0.0;

bool left_rotation = false;
bool right_rotation = false;


void leftForceCallback(const std_msgs::Float64MultiArrayConstPtr& last_msg){
    std_msgs::Float64MultiArray msg;
    geometry_msgs::WrenchStamped left_sensor_force;
    //data: force-distance_x-distance_y
    msg.data.push_back(last_msg->data[0]-1.0);
    msg.data.push_back(last_msg->data[2]);
    msg.data.push_back(last_msg->data[3]);

    float Mx = msg.data[0]*(msg.data[2]-2.8);
    float My = msg.data[0]*(msg.data[1]-5.57);

    left_sensor_force.header.stamp = ros::Time::now();
    left_sensor_force.header.frame_id = "retractor";
    left_sensor_force.wrench.force.x = msg.data[0];
    left_sensor_force.wrench.force.y = 0.0;
    left_sensor_force.wrench.force.z = 0.0;
    left_sensor_force.wrench.torque.x = 0.0;

    if(left_rotation){
        left_sensor_force.wrench.torque.z = -Mx;
        left_sensor_force.wrench.torque.y = -My/2;
    }else{
        left_sensor_force.wrench.torque.y = 0.0;
        left_sensor_force.wrench.torque.z = 0.0;
    }


    if(std::abs(Mx/2) < left_MX){
        left_sensor_force.wrench.torque.z = 0.0;
    }
    if(std::abs(My/2) < left_MY){
        left_sensor_force.wrench.torque.y = 0.0;
    }
    if(left_force_data_recorder==1){
        output_left << msg.data[0] << "\t";
        output_left << Mx << "\t";
        output_left << My << endl;
    }

    
    left_force_pub.publish(left_sensor_force);
}

void rightForceCallback(const std_msgs::Float64MultiArrayConstPtr& last_msg){

    std_msgs::Float64MultiArray msg;
    geometry_msgs::WrenchStamped right_sensor_force;

    msg.data.push_back(last_msg->data[1]-1.0);
    msg.data.push_back(last_msg->data[4]);
    msg.data.push_back(last_msg->data[5]);

    float Mx = msg.data[0]*(msg.data[2]-2.8);
    float My = msg.data[0]*(msg.data[1]-5.57);

    right_sensor_force.header.stamp = ros::Time::now();
    right_sensor_force.header.frame_id = "retractor";
    right_sensor_force.wrench.force.x = msg.data[0];
    right_sensor_force.wrench.force.y = 0.0;
    right_sensor_force.wrench.force.z = 0.0;
    right_sensor_force.wrench.torque.x = 0.0;

    if(right_rotation){
        right_sensor_force.wrench.torque.z = -Mx;
        right_sensor_force.wrench.torque.y = My/2;
    }else{
        right_sensor_force.wrench.torque.y = 0.0;
        right_sensor_force.wrench.torque.z = 0.0;
    }
    if(std::abs(Mx/2) < right_MX){
        right_sensor_force.wrench.torque.z = 0.0;
    }
    if(std::abs(My/2) < right_MY){
        right_sensor_force.wrench.torque.y = 0.0;
    }

    if(right_force_data_recorder==1){
        output_right << msg.data[0] << "\t";
        output_right << Mx << "\t";
        output_right << My << endl;
    }

    
    right_force_pub.publish(right_sensor_force);
}

void configCallback(ur5e_test::referenceConfig &config, uint32_t level){

    left_force_config = config.left_force;
    right_force_config = config.right_force;
    left_rotation = config.left_rotation;
    right_rotation = config.right_rotation;
    left_MX = config.left_MX;
    left_MY = config.left_MY;
    right_MX = config.right_MX;
    right_MY = config.right_MY;

    left_force_data_recorder = config.left_force_recorder;
    right_force_data_recorder = config.right_force_recorder;
}


int main(int argc, char **argv){

    ros::init(argc, argv,"force");
    ros::NodeHandle nh;

    //subscribe force data from serial com 
    ros::Subscriber left_force_sub = nh.subscribe("/touch_force", 1, &leftForceCallback);
    ros::Subscriber right_force_sub = nh.subscribe("/touch_force", 1, &rightForceCallback);

    //publish reference force to special topic
    ros::Publisher ref_force_pub_left = nh.advertise<geometry_msgs::WrenchStamped>("/left/cartesian_force_controller/target_wrench", 1);
    ros::Publisher ref_force_pub_right = nh.advertise<geometry_msgs::WrenchStamped>("/right/cartesian_force_controller/target_wrench", 1);

    //publish force to ft_sensor topic
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

    //dynamic_reconfigure callback setting;
    dynamic_reconfigure::Server<ur5e_test::referenceConfig> server;
    dynamic_reconfigure::Server<ur5e_test::referenceConfig>::CallbackType f;
    f=boost::bind(&configCallback, _1, _2);
    server.setCallback(f);

    output_left.open("/home/jwshen-310/Code/RAL/data/left_force.txt");
    output_right.open("/home/jwshen-310/Code/RAL/data/right_force.txt");

    while(ros::ok()){

        ref_force_left.wrench.force.x = -left_force_config;
        ref_force_right.wrench.force.x = -right_force_config;

        ref_force_pub_left.publish(ref_force_left);
        ref_force_pub_right.publish(ref_force_right);
        ros::spinOnce();
        loop_rate.sleep();
    }

}