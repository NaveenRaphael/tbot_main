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
/*
See Swarm Motion: 
This code is "deprecated"
*/

void set(point2D p, tbot_main::positionRequest &t)
{
    t.request.x = p.x;
    t.request.y = p.y;
}

int main(int argc, char *argv[])
{
    // Initialisation
    int num;     //Number of bots; taken from the launch file
    float r = 1; //Radius of "circle"; should be
    float x, y;  //Center of circle
    ros::init(argc, argv, "follow_path");
    ros::NodeHandle n;
    //Getting parameters from the launch file
    n.getParam("num", num);
    n.getParam("rad", r);
    n.getParam("cy", y);
    n.getParam("cx", x);

    ROS_INFO("%d", num);

    point2D o = point2D(x, y); //Origin

    ros::ServiceClient cmd_pos[num]; //making num number of service clients
    for (int i = 0; i < num; i++)
    {
        char name[25];
        sprintf(name, "/tb3_%d/change_tbot_pos", i);
        cmd_pos[i] = n.serviceClient<tbot_main::positionRequest>(name);
        ROS_INFO("%s", name);
    }

    tbot_main::positionRequest t;   //Temporary variable to store publishing data

    float ang = 0;                  //Present "orientation" of swarm;
    float ch = 2 * M_PI / num;      //phase difference between 2 bots
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ang = ang + EPS;                //Present "orientation" of swarm; swarm rotates by angle EPS here;
        for (int i = 0; i < num; i++)   //Edit the position and orientation of each of the bot. 
        {
            set(o - point2D(-cos(ang + ch * i) * r, -sin(ang + ch * i) * r), t);        //For circle
            cmd_pos[i].call(t);         //Call Service to publish data
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}