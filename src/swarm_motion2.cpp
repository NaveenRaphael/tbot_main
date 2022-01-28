#include <ros/ros.h>
#include "tbot_main/velocityRequest.h"
#include "tbot_main/trajectory.h"
#include <geometry_msgs/Twist.h>
#include "point.hpp"
#include "softbody.hpp"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#define EPS 0.01


/*
Will replace old file


*/
class form_trajectory{
    SoftBody2& primary;
    int number;
    ros::Publisher trajectory;
    tf::TransformBroadcaster& br;
    ros::NodeHandle& n;

    point2D peek(double);
public:
    form_trajectory(SoftBody2&, ros::NodeHandle&, tf::TransformBroadcaster&, int);
    void publish();
    tbot_main::trajectory get_trajectory();
};
form_trajectory::form_trajectory(SoftBody2& p, ros::NodeHandle& n_, tf::TransformBroadcaster& b, int num): 
primary(p), 
n(n_),
br(b)
{
    number=num;

    char name[25];
    sprintf(name, "/tb3_%d/trajectory", number);
    trajectory= n.advertise<tbot_main::trajectory>(name, 1000);
}
point2D form_trajectory::peek(double time){
    return primary.get_ith(number, time);
}
tbot_main::trajectory form_trajectory::get_trajectory(){
    double time_now =ros::Time::now().toSec();

    point2D nextPoint = peek(time_now+EPS), prevPoint =peek(time_now-EPS), presPoint=peek(time_now);

    double orientation = (nextPoint-prevPoint).get_angle();
    double vel=sqrt((nextPoint-prevPoint).norm2())/(2*EPS);
    double w = s2dis((nextPoint-presPoint).get_angle(),(presPoint-prevPoint).get_angle())/EPS
    ;

    tbot_main::trajectory ret;
    ret.x=presPoint.x;
    ret.y=presPoint.y;
    ret.w=w;
    ret.t=orientation;
    ret.v=vel;

    return ret;


}
void form_trajectory::publish(){

    tbot_main::trajectory to_pub =get_trajectory();
    trajectory.publish(to_pub);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(to_pub.x, to_pub.y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, to_pub.t);
    transform.setRotation(q);

    char name[25];
    sprintf(name, "/tb3_%d_togo", number);

    // ROS_INFO("v:%.3f, w: %.3f", to_pub.v, to_pub.w);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));

}


int main(int argc, char *argv[])
{
    int number_of_bot=2;
    ros::init(argc, argv, "Circle_swarm");
    ros::NodeHandle n;
    auto p=ros::Time::now();
    static tf::TransformBroadcaster br;

    SoftBody2 primary(3, p.toSec());

    std::vector<form_trajectory> publishers;

    for (int i = 0; i < number_of_bot; i++)
    {
        publishers.push_back(form_trajectory(primary, n,br, i));
    }

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        for(auto el:publishers){
            el.publish();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}
