#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>

class display{
    public:
        display(){
            sub_force = nh.subscribe("/touch_force", 1, &display::callback, this);
            pub_force = nh.advertise<std_msgs::Float64MultiArray>("/touch_force_data",1);
        }
        void callback(const std_msgs::Float64MultiArrayConstPtr& last_msg){
            std_msgs::Float64MultiArray msg;
            msg.data.push_back(last_msg->data[0]-1.0);
            msg.data.push_back(last_msg->data[1]-1.0);
            // output << msg.data[0] << "\t";
            // output << msg.data[1] << std::endl;
            pub_force.publish(msg);
        
        }

        //std::ofstream output;
    
    private:
        ros::NodeHandle nh;

        ros::Publisher pub_force;
        ros::Subscriber sub_force;


};


int main(int argc, char** argv){
    ros::init(argc, argv, "display");
    display display;
    //display.output.open("/home/jwshen-310/force.txt");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown(); 
  //ros::spin();

    return 0;

}