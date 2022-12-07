#include "sigma_client/forward_kinematic.h"

forward_kinematic::forward_kinematic(){
    initMatrix();
    joint_sub = nh.subscribe("/right/joint_states", 1, &forward_kinematic::jointStateCallback, this);
    cartesian_pub = nh.advertise<geometry_msgs::PoseStamped>("/mycurrent_pose", 1);
}

void forward_kinematic::initMatrix(){
    T01 = Eigen::Matrix4d::Identity();
    T12 = Eigen::Matrix4d::Identity();
    T23 = Eigen::Matrix4d::Identity();
    T34 = Eigen::Matrix4d::Identity();
    T45 = Eigen::Matrix4d::Identity();
    T56 = Eigen::Matrix4d::Identity();
    T_target = Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d forward_kinematic::T_Matrix(int index){

    Eigen::Matrix4d T_i = Eigen::Matrix4d::Identity();
    T_i(0,0) = cos(q_current[index]);
    T_i(0,1) = -sin(q_current[index])*cos(dh.alpha[index]);
    T_i(0,2) = sin(q_current[index])*sin(dh.alpha[index]);
    T_i(0,3) = dh.a[index]*cos(q_current[index]);
    T_i(1,0) = sin(q_current[index]);
    T_i(1,1) = cos(q_current[index])*cos(dh.alpha[index]);
    T_i(1,2) = -cos(q_current[index])*sin(dh.alpha[index]);
    T_i(1,3) = dh.a[index]*sin(q_current[index]);
    T_i(2,1) = sin(dh.alpha[index]);
    T_i(2,2) = cos(dh.alpha[index]);
    T_i(2,3) = dh.d[index];

    return T_i;
}

Eigen::Matrix4d forward_kinematic::calculateTMatrix(){

    T01 = T_Matrix(0);
    T12 = T_Matrix(1);
    T23 = T_Matrix(2);
    T34 = T_Matrix(3);
    T45 = T_Matrix(4);
    T56 = T_Matrix(5);
    T_target = T01 * T12 * T23 * T34 * T45 * T56;

    return T_target;
}

void forward_kinematic::jointStateCallback(const sensor_msgs::JointStateConstPtr &state){

    for(int i = 0; i < 6; i++){
        q_current[i] = state->position[i];
    }
    posePublish();
}

std::vector<float> forward_kinematic::RotToQuaternions(Eigen::Matrix4d &T){

    float m[3][3];
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            m[i][j] = T(i,j);
        }
    }

    float w,x,y,z;
    std::vector<float> res(4,0);

    //探测四元数中最大的项 
    float fourWSquaredMinusl = m[1][1]+m[2][2]+m[3][3];
    float fourXSquaredMinusl = m[1][1]-m[2][2]-m[3][3];
    float fourYSquaredMinusl = m[2][2]-m[1][1]-m[3][3];
    float fourZSquaredMinusl = m[3][3]-m[1][1]-m[2][2];

    int biggestIndex = 0;
    float fourBiggestSqureMinus1 = fourWSquaredMinusl;
    if(fourXSquaredMinusl>fourBiggestSqureMinus1){
        fourBiggestSqureMinus1 = fourXSquaredMinusl;
        biggestIndex =1;
    } 
    if(fourYSquaredMinusl>fourBiggestSqureMinus1){
        fourBiggestSqureMinus1 = fourYSquaredMinusl;
        biggestIndex =2;
    } 
    if(fourZSquaredMinusl>fourBiggestSqureMinus1){
        fourBiggestSqureMinus1 = fourZSquaredMinusl;
        biggestIndex =3;
    } 

    //计算平方根和除法 
    float biggestVal = sqrt(fourBiggestSqureMinus1+1.0f)*0.5f;
    float mult = 0.25f/biggestVal;

    //计算四元数的值
    switch(biggestIndex){
        case 0:
            w=biggestVal;
            x=(m[2][3]-m[3][2])*mult;
            y=(m[3][1]-m[1][3])*mult;
            z=(m[1][2]-m[2][1])*mult;
            break;
        case 1:
            x = biggestVal;
            w =(m[2][3]-m[3][2])*mult;
            y =(m[1][2]+m[2][1])*mult;
            z =(m[3][1]+m[1][3])*mult;
            break;
        case 2:
            y =biggestVal;
            w =(m[3][1]-m[1][3])*mult;
            x =(m[1][2]+m[2][1])*mult;
            z =(m[2][3]+m[3][2])*mult;
            break;
        case 3:
            z =biggestVal;
            w =(m[1][2]-m[2][1])*mult;
            x =(m[3][1]+m[1][3])*mult;
            y =(m[2][3]+m[3][2])*mult;
            break;
    }

    res[0] = w;
    res[1] = x;
    res[2] = y;
    res[3] = z;

    return res;
    
}

void forward_kinematic::posePublish(){

    Eigen::Matrix4d T_current = calculateTMatrix();
    std::vector<float> quaternions = RotToQuaternions(T_current);

    geometry_msgs::PoseStamped pose_current;
    pose_current.header.frame_id = "";
    pose_current.header.stamp = ros::Time::now();
    pose_current.pose.position.x = T_current(0,3)/1000;
    pose_current.pose.position.y = T_current(1,3)/1000;
    pose_current.pose.position.z = T_current(2,3)/1000;
    pose_current.pose.orientation.x = quaternions[1];
    pose_current.pose.orientation.y = quaternions[2];
    pose_current.pose.orientation.z = quaternions[3];
    pose_current.pose.orientation.w = quaternions[0];

    cartesian_pub.publish(pose_current);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "forward_kinematic");
    forward_kinematic fk;

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;

}