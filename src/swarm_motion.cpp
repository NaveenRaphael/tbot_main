#include <ros/ros.h>
#include "tbot_main/velocityRequest.h"
#include "tbot_main/positionRequest.h"
#include <geometry_msgs/Twist.h>
#include "point.hpp"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
// #include <format>
#define EPS 0.01

void set(point2D p, tbot_main::positionRequest &t)
{
    t.request.x = p.x;
    t.request.y = p.y;
}

class SoftBody
{
    point2D center;
    float orientation;
    int num;

    point2D velocityCOM;
    float omega;

    ros::Time last_time, present;
    double diff;

    double getR(int);
    point2D getDir(int);

public:
    SoftBody(int);
    void update();
    point2D getIth(int);
};

SoftBody::SoftBody(int n)
{
    num = n;

    //Center:
    center.set(0, 0);
    orientation = 0;

    velocityCOM.set(0, 0);
    omega = 0.05;

    last_time = ros::Time::now();
}

void SoftBody::update()
{
    present = ros::Time::now();
    diff = (present - last_time).toSec();
    last_time = present;

    center = center + velocityCOM * diff;
    orientation += omega * diff;

    velocityCOM = velocityCOM;
    omega = omega;
}

point2D SoftBody::getDir(int n)
{
    //Simple rn:
    double ang = 2.0*n / num  * M_PI + orientation;
    return point2D(cos(ang), sin(ang));
}

double SoftBody::getR(int n)
{
    //simple rn
    return 0.5;
}

point2D SoftBody::getIth(int n)
{
    return center + getDir(n) * getR(n);
}

int main(int argc, char *argv[])
{
    // Initialisation
    int num; //Number of bots; taken from the launch file

    ros::init(argc, argv, "general_swarm");
    ros::NodeHandle n;
    //Getting parameters from the launch file
    n.getParam("num", num);

    SoftBody swarm(num);

    ros::ServiceClient cmd_pos[num]; //making num number of service clients
    for (int i = 0; i < num; i++)
    {
        char name[25];
        sprintf(name, "/tb3_%d/change_tbot_pos", i);
        cmd_pos[i] = n.serviceClient<tbot_main::positionRequest>(name);
        ROS_INFO("%s", name);
    }

    tbot_main::positionRequest t; //Temporary variable to store publishing data

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        swarm.update();
        for (int i = 0; i < num; i++) //Edit the position and orientation of each of the bot.
        {
            set(swarm.getIth(i), t);
            cmd_pos[i].call(t); //Call Service to publish data
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}