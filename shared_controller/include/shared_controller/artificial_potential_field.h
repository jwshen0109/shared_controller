#ifndef ARTIFICIAL_POTENTIAL_FIELD_H_
#define ARTIFICIAL_POTENTIAL_FIELD_H_

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>

#include <iostream>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

using namespace std;

class Point
{
public:
    Point(float _x, float _y, float _z) : x(_x), y(_y), z(_z)
    {
    }

public:
    float x;
    float y;
    float z;
};

class Cylinder
{
public:
    Cylinder(float _x, float _y, float _R) : x(_x), y(_y), R(_R)
    {
    }

public:
    float x;
    float y;
    float R;
};

class forceVector
{
public:
    forceVector(Point p1, Point p2) : p_start(p1), p_end(p2)
    {
        length = sqrt(pow((p_end.x - p_start.x), 2) + pow(p_end.y - p_start.y, 2));
    };

    forceVector addVector(forceVector v1, forceVector v2);

public:
    Point p_start;
    Point p_end;
    float length;
};

class APF
{
public:
    APF();
    ~APF();

    float distanceCalculation(Point &p1, Point &p2);

    forceVector forceAttraction(Point &p_target, Point &p_current);

    forceVector forceRepulsion();

    float porcess(float cur_velocity, Point &p_target, Point &p_current);

public:
    // to do

private:
    float dis_att;
    float eta = 0.3;
    float delta_t = 0.005;
};

class VirtualFixture
{
public:
    VirtualFixture();
    vector<float> minDistancePoint(Point &p0, Cylinder &C0);
    void PublishVirtualForce(Point &p0, Cylinder &C0, vector<float> &velocity);

public:
    ros::NodeHandle nh;
    ros::Publisher vfForce_pub;

    float eta_p = 20;
    float eta_v = 20;
};

#endif // _ARTIFICIAL_POTENTIAL_FIELD_H