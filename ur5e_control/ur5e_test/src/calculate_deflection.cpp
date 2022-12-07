#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <vector>
#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>

using namespace std;
using namespace Eigen;

class Calculator{
    public:

        Calculator(ros::NodeHandle nh){
            delta_distance_sub = nh.subscribe("/delta_distance", 1, &Calculator::distanceCallback, this);
            euler_pub = nh.advertise<std_msgs::Float64MultiArray>("/final_euler", 1);
        }

        void rotationAngle(vector<double>& delta_distance){
            
            Matrix3Xd input(3, 4), output(3, 4);
            for(int i=0; i<4; i++){
                input(2, i) = 0;
            }
            input(0, 0) = 5.5;
            input(0, 1) = -5.5;
            input(0, 2) = -5.5;
            input(0, 3) = 5.5;
            input(1, 0) = 2.8;
            input(1, 1) = 2.8;
            input(1, 2) = -2.8;
            input(1, 3) = -2.8;
            output = input;
            output(2,0) = delta_distance[0];
            output(2,1) = delta_distance[1];
            output(2,2) = delta_distance[2];
            output(2,3) = delta_distance[3];
            Eigen::Matrix3d rotation = Find3DRotationTransform(input, output);
            Eigen::Vector3d euler = rotation.eulerAngles(0,1,2);
            for(int i=0; i<3; i++){
                final_euler[i] = euler(i);
            }
            calculation_flag = 1;
        }

        Eigen::Matrix3d Find3DRotationTransform(Matrix3Xd P, Matrix3Xd Q) {

        // Default output
            Affine3d A;
            A.linear() = Matrix3d::Identity(3, 3);
            A.translation() = Vector3d::Zero();

            if (P.cols() != Q.cols())
                throw "Find3DAffineTransform(): input data mis-match";

            // Center the data
            Vector3d p = P.rowwise().mean();
            Vector3d q = Q.rowwise().mean();

            Matrix3Xd X = P.colwise() - p;
            Matrix3Xd Y = Q.colwise() - q;
            ROS_INFO("SVD starting");
            // SVD
            MatrixXd Cov = X*Y.transpose();
            JacobiSVD<MatrixXd> svd(Cov, ComputeThinU | ComputeThinV);

            // Find the rotation, and prevent reflections
            Matrix3d I = Matrix3d::Identity(3, 3);
            double d = (svd.matrixV()*svd.matrixU().transpose()).determinant();
            (d > 0.0) ? d = 1.0 : d = -1.0;
            I(2, 2) = d;
            ROS_INFO("Rotation Matrix generating");
            Matrix3d R = svd.matrixV()*I*svd.matrixU().transpose();

            // The final transform
            //   A.linear() = R;
            //   A.translation() = q - R*p;

            return R;
        }

        void distanceCallback(const std_msgs::Float64MultiArrayConstPtr& msg){
            std_msgs::Float64MultiArray euler_msgs;

            vector<double> delta_distance(4,0.0);
            for(int i=0; i<4; i++){
                delta_distance[i] = msg->data[i];
            }
            ROS_INFO("calculating");
            rotationAngle(delta_distance);
            ROS_INFO("calculation is done");
            for(int i=0; i<3; i++){
                double result = final_euler[i]*180/3.1415926;
                if(abs(result)>140){
                    result = result>0?(180-result):(180+result);
                }
                euler_msgs.data.push_back(result);
            }
            if(calculation_flag==1){
                euler_pub.publish(euler_msgs);
                ROS_INFO("final_euler publish success");
            }
        }

    private:

        ros::Subscriber delta_distance_sub;
        ros::Publisher euler_pub;

        int calculation_flag = 0;
        vector<double> final_euler = vector<double>(3,0.0);

};

int main(int argc, char **argv){

    ros::init(argc, argv,"calculator");
    ros::NodeHandle nh;
    Calculator* calculator = new Calculator(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown(); 

    return 0;

}

