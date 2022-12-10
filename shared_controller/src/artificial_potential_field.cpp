#include "shared_controller/artificial_potential_field.h"

forceVector forceVector::addVector(forceVector v1, forceVector v2)
{
    Point p_tmp(v2.p_end.x + v1.p_end.x - v1.p_start.x, v2.p_end.y + v1.p_end.y - v1.p_start.y, v2.p_end.z + v1.p_end.z - v1.p_start.z);
    forceVector vt(v1.p_start, p_tmp);
    return vt;
}

APF::APF()
{
    //
}

float APF::distanceCalculation(Point &p1, Point &p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

forceVector APF::forceAttraction(Point &p_target, Point &p_current)
{
    dis_att = distanceCalculation(p_target, p_current);
    double force_att = eta * dis_att;
    Point p_Fatt(force_att / dis_att * (p_target.x - p_current.x) + p_current.x, force_att / dis_att * (p_target.y - p_current.y) + p_current.y, force_att / dis_att * (p_target.z - p_current.z) + p_current.z);
    forceVector vFatt(p_current, p_Fatt);
    return vFatt;
}

forceVector APF::forceRepulsion()
{
}

float APF::porcess(float cur_velocity, Point &p_target, Point &p_current)
{
    forceVector vFtotal = forceAttraction(p_target, p_current); // attractive force
    float dis = cur_velocity * delta_t + vFtotal.length * pow(delta_t, 2) / 2;

    return dis;
}

VirtualFixture::VirtualFixture()
{
    // TODO:
    vfForce_pub = nh.advertise<geometry_msgs::WrenchStamped>("/sigma/force_feedback", 1);
}

vector<float> VirtualFixture::minDistancePoint(Point &p0, Cylinder &C0)
{
    vector<float> p_result(3, 0.0);
    float d_r = sqrt(pow(p0.x - C0.x, 2) + pow(p0.y - C0.y, 2));
    float dis_min = d_r - C0.R;
    if (dis_min < 0)
    {
        return p_result;
    }
    float dis_min_x = (p0.x - C0.x) / d_r * dis_min;
    float dis_min_y = (p0.y - C0.y) / d_r * dis_min;
    p_result[0] = dis_min_x;
    p_result[1] = dis_min_y;

    return p_result;
}

void VirtualFixture::PublishVirtualForce(Point &p0, Cylinder &C0)
{
    vector<float> p_result = minDistancePoint(p0, C0);
    geometry_msgs::WrenchStamped virtual_force;

    virtual_force.header.frame_id = "base_link";
    virtual_force.header.stamp = ros::Time::now();

    virtual_force.wrench.force.x = p_result[0] * eta;
    virtual_force.wrench.force.y = p_result[1] * eta;
    virtual_force.wrench.force.z = p_result[2] * eta;
    virtual_force.wrench.torque.x = 0.0;
    virtual_force.wrench.torque.y = 0.0;
    virtual_force.wrench.torque.z = 0.0;
    vfForce_pub.publish(virtual_force);
}