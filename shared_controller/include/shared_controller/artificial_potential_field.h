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

class Point3D
{
public:
    Point3D(float _x, float _y, float _z) : x(_x), y(_y), z(_z)
    {
    }

public:
    float x;
    float y;
    float z;
};

class Point2D
{
public:
    Point2D(float _x, float _y) : x(_x), y(_y)
    {
    }

public:
    float x;
    float y;
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
    forceVector(Point2D p1, Point2D p2) : p_start(p1), p_end(p2)
    {
        length = sqrt(pow((p_end.x - p_start.x), 2) + pow(p_end.y - p_start.y, 2));
    };

    forceVector addVector(forceVector v1, forceVector v2);

public:
    Point2D p_start;
    Point2D p_end;
    float length;
};

class APF
{
public:
    APF();
    ~APF();

    float distanceCalculation(Point2D &p1, Point2D &p2);

    forceVector forceAttraction(Point2D &p_target, Point2D &p_current);

    forceVector xFedge(Point2D &retractor_cur, float v_x);
    forceVector yFedge(Point2D &retractor_cur, float v_y);

    float Process(Point2D &retractor_cur, vector<float> vel_cur);

public:
    // to do

private:
    float dis_att;
    float eta_att = 0.3;
    float eta_rep = 0.02;
    float delta_t = 0.005;

    // retractor size
    float retractor_width = 0.015;
    float retractor_height = 0.035;
    // edge size
    float edge_width = 0.03;
    float edge_height = 0.05;
};

class VirtualFixture
{
public:
    VirtualFixture();
    vector<float> minDistancePoint(Point2D &p0, Cylinder &C0);
    void PublishVirtualForce(Point2D &p0, Cylinder &C0, vector<float> &velocity);

public:
    ros::NodeHandle nh;
    ros::Publisher vfForce_pub;

    float eta_p = 20;
    float eta_v = 15;
};

#endif // _ARTIFICIAL_POTENTIAL_FIELD_H