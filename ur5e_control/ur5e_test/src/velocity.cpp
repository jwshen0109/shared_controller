#include <ros/ros.h>
#include <cartesian_motion_controller/velocity.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>

class velocity{
    public:
        velocity(){
            velocity_1 = nh.subscribe("/left/current_position", 1, &velocity::velCallback1, this);
            velocity_2 = nh.subscribe("/right/current_velocity", 1, &velocity::velCallback2, this);
        }
        void velCallback1(const cartesian_motion_controller::velocityConstPtr & last_velocity){
            data[0]=last_velocity->twist.linear.x * 100.0;
            data[1]=last_velocity->twist.linear.y * 100.0;
            data[2]=last_velocity->twist.linear.z * 100.0;
            output << data[0] << "\t";
            output << data[1] << "\t";
            output << data[2] << std::endl;
        }
        void velCallback2(const cartesian_motion_controller::velocityConstPtr & last_velocity){
            //data[1] = last_velocity->twist.linear.x * 100.0;
        }

        std::ofstream output;
    
    private:
        ros::NodeHandle nh;
        ros::Subscriber velocity_1;
        ros::Subscriber velocity_2;
        double data[3];
};

int main(int argc, char **argv){
    ros::init(argc, argv,"vel");
    velocity velocity;
    velocity.output.open("/home/jwshen-310/velocity.txt");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown(); 
    //ros::spin();

    return 0;
}