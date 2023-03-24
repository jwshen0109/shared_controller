#include "shared_controller/artificial_potential_field.h"

constexpr auto e = 2.71828;

forceVector forceVector::addVector(forceVector v1, forceVector v2)
{
    Point2D p_tmp(v2.p_end.x + v1.p_end.x - v1.p_start.x, v2.p_end.y + v1.p_end.y - v1.p_start.y);
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

//
forceVector APF::xFedge(Point2D &retractor_cur, float v_x)
{
    double Fedge = 0.0;
    Point2D pFedge(0.0, 0.0);
    if (-(edge_width - retractor_width / 2) <= retractor_cur.x && retractor_cur.x < 0.0)
    {
        Fedge = eta_repulsion * pow(retractor_cur.x, 2) / 3;
        pFedge = Point2D(retractor_cur.x + Fedge, retractor_cur.y);
    }
    else if (retractor_cur.x >= 0.0 && retractor_cur.x <= (edge_width - retractor_width / 2))
    {
        Fedge = eta_repulsion * pow(retractor_cur.x, 2) / 3;
        pFedge = Point2D(retractor_cur.x - Fedge, retractor_cur.y);
    }
    else if (-edge_width < retractor_cur.x && retractor_cur.x < -(edge_width - retractor_width / 2))
    {
        Fedge = eta_repulsion * v_x * pow(e, -retractor_cur.x);
        pFedge = Point2D(retractor_cur.x + Fedge, retractor_cur.y);
    }
    else if ((edge_width - retractor_width / 2) < retractor_cur.x && retractor_cur.x < edge_width)
    {
        Fedge = eta_repulsion * v_x * pow(e, retractor_cur.x);
        pFedge = Point2D(retractor_cur.x - Fedge, retractor_cur.y);
    }

    forceVector vFedge(retractor_cur, pFedge);
    vFedge.printVector("vFedge");
    // vFedge.drawVector();

    return vFedge;
}

forceVector APF::yFedge(Point2D &retractor_cur, float v_y)
{
    double Fedge = 0.0;
    Point2D pFedge(0.0, 0.0);
    if (-(edge_height - retractor_height / 2) <= retractor_cur.y && retractor_cur.y < 0.0)
    {
        Fedge = eta_repulsion * pow(retractor_cur.y, 2) / 3;
        pFedge = Point2D(retractor_cur.y + Fedge, retractor_cur.x);
    }
    else if (retractor_cur.y >= 0.0 && retractor_cur.y <= (edge_height - retractor_height / 2))
    {
        Fedge = eta_repulsion * pow(retractor_cur.y, 2) / 3;
        pFedge = Point2D(retractor_cur.y - Fedge, retractor_cur.x);
    }
    else if (-edge_height < retractor_cur.y && retractor_cur.y < -(edge_height - retractor_height / 2))
    {
        Fedge = eta_repulsion * v_y * pow(e, -retractor_cur.y);
        pFedge = Point2D(retractor_cur.y + Fedge, retractor_cur.x);
    }
    else if ((edge_height - retractor_height / 2) < retractor_cur.y && retractor_cur.y < edge_height)
    {
        Fedge = eta_repulsion * v_y * pow(e, retractor_cur.y);
        pFedge = Point2D(retractor_cur.y - Fedge, retractor_cur.x);
    }

    forceVector vFedge(retractor_cur, pFedge);
    vFedge.printVector("vFedge");
    // vFedge.drawVector();

    return vFedge;
}

float APF::Process(Point2D &retractor_cur, vector<float> vel_cur)
{
    forceVector vxFtotal = xFedge(retractor_cur, vel_cur[0]);
    forceVector vyFtotal = yFedge(retractor_cur, vel_cur[1]);
    float dis = vel_cur * delta_t + vFtotal.length * pow(delta_t, 2) / 2;

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

void VirtualFixture::PublishVirtualForce(Point &p0, Cylinder &C0, vector<float> &velocity)
{
    vector<float> p_result = minDistancePoint(p0, C0);
    geometry_msgs::WrenchStamped virtual_force;

    virtual_force.header.frame_id = "base_link";
    virtual_force.header.stamp = ros::Time::now();

    virtual_force.wrench.force.x = p_result[0] * eta_p + velocity[0] * eta_v;
    virtual_force.wrench.force.y = p_result[1] * eta_p + velocity[1] * eta_v;
    virtual_force.wrench.force.z = p_result[2] * eta_p + velocity[2] * eta_v;
    virtual_force.wrench.torque.x = 0.0;
    virtual_force.wrench.torque.y = 0.0;
    virtual_force.wrench.torque.z = 0.0;
    vfForce_pub.publish(virtual_force);
}