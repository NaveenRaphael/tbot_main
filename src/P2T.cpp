/*
Converts from recieving points to trajectories
Needs cleaning
*/

#include <ros/ros.h>
#include "tbot_main/velocityRequest.h"
#include "tbot_main/positionRequest.h"
#include "tbot_main/trajectory.h"
#include <geometry_msgs/Twist.h>
#include "point.hpp"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

/**
 * @brief A class to convert a list of points to the trajectory form needed for trajectory tracking code.
 *
 */
class P2T
{
private:
    point2D prev;
    double prev_time;
    double v;
    double w;
    double pTheta;
    bool store_position(tbot_main::positionRequest::Request &r,
                        tbot_main::positionRequest::Response &g);
    ros::ServiceServer change_pos;
    ros::NodeHandle n;
    ros::Publisher trajectory;

public:
    P2T();
};

/**
 * @brief converts position requests to trajectory requests
 *
 * @param r Position to go
 * @param g true
 * @return true always, needed by ros, it seems
 * @return false
 */
bool P2T::store_position(tbot_main::positionRequest::Request &r,
                         tbot_main::positionRequest::Response &g)
{
    // Define new values:
    point2D present = point2D(r.x, r.y);
    double theta = (present - prev).get_angle();
    double cur_time = ros::Time::now().toSec();
    double delta = cur_time - prev_time;

    v = sqrt((present - prev).norm2()) / (cur_time - prev_time);
    w = s2dis(theta, pTheta) / (delta);

    tbot_main::trajectory traj;
    traj.x = present.x;
    traj.y = present.y;
    traj.v = v;
    traj.w = w;
    traj.t = theta;

    pTheta = theta;
    prev = present;
    prev_time = cur_time;

    trajectory.publish(traj);

    return true;
}
/**
 * @brief Construct a new P2T::P2T object
 *
 */
P2T::P2T() : n()
{
    trajectory = n.advertise<tbot_main::trajectory>("trajectory", 1000);
    change_pos = n.advertiseService("change_tbot_pos", &P2T::store_position, this);
    prev_time = ros::Time::now().toSec();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "P2T");
    P2T test;

    ros::spin();
    return 0;
}