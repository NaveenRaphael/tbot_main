#include <ros/ros.h>
#include "tbot_main/velocityRequest.h"
#include "tbot_main/trajectory.h"
#include <geometry_msgs/Twist.h>
#include "point.hpp"
#include "softbody.hpp"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <functional>

point2D roundedSquare(double);

#define EPS 0.01
/**
 * @brief A better trajectory controlling code
 *
 */

/**
 * @brief A class that handles the trajectory of a single robot, provided as a lambda function in the constructor
 *
 * num stores the number of the tbot that we are controlling.
 *
 */
class trajectory
{
    std::function<point2D(double)> peek;
    ros::Time start;
    const int num;
    ros::NodeHandle &n;
    ros::Publisher pub;
    tf::TransformBroadcaster &br;
    tbot_main::trajectory get_trajectory()
    {
        double t = (ros::Time::now() - start).toSec();

        point2D nextPoint = peek(t + EPS), prevPoint = peek(t - EPS), presPoint = peek(t);

        double orientation = (nextPoint - prevPoint).get_angle();
        double vel = sqrt((nextPoint - prevPoint).norm2()) / (2 * EPS);
        double w = s2dis((nextPoint - presPoint).get_angle(), (presPoint - prevPoint).get_angle()) / EPS;

        tbot_main::trajectory ret;
        ret.x = presPoint.x;
        ret.y = presPoint.y;
        ret.w = w;
        ret.t = orientation;
        ret.v = vel;

        return ret;
    }

public:
    /**
     * @brief Construct a new trajectory object
     *
     * @param i Number of tbot
     * @param nh node handler passed from the main
     * @param br_ broadcaster passed from the main
     * @param q of the form q: double -> point2D, given a time t, returns the point the robot is supposed to be.
     */
    trajectory(int i, ros::NodeHandle &nh, tf::TransformBroadcaster &br_, std::function<point2D(double)> q) : num(i), peek(q), n(nh), br(br_)
    {
        start = ros::Time::now();
        char name[25];
        sprintf(name, "/tb3_%d/trajectory", num);
        pub = n.advertise<tbot_main::trajectory>(name, 1000);
    }
    /**
     * @brief Publishes in both tf and service
     *
     */
    void publish()
    {
        tbot_main::trajectory to_pub = get_trajectory();
        pub.publish(to_pub);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(to_pub.x, to_pub.y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, to_pub.t);
        transform.setRotation(q);

        char name[25];
        sprintf(name, "/tb3_%d_togo", num);

        // ROS_INFO("v:%.3f, w: %.3f", to_pub.v, to_pub.w);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "New_swarm");
    ros::NodeHandle n;
    tf::TransformBroadcaster br;

    std::array<trajectory, 2> swarms{
        trajectory(1, n, br, [](double t)
                   {
            double phase = 0.2 * t;
            return point2D(0.5, 0.5) + point2D(sin(phase), cos(phase)) * 0.5; }),

        trajectory(3, n, br, roundedSquare)};

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        for (auto el : swarms)
        {
            el.publish();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/**
 * Some basic trajectories:
 *Circle
[](double t){
double phase=omega *t;
return point2D(center.x, center.y)+ point2D(sin(phase), cos(phase))*radius;}
 *
 */

point2D square(double t)
{
    double time_for_one_side = 3;
    double side_length = 1.5;
    point2D start(0, 0);
    point2D side_1(side_length, 0);
    point2D side_2(0, side_length);
    int count = int(t / time_for_one_side);
    double leftover = (t - count * time_for_one_side);
    switch (count % 4)
    {
    case 0:
        return start + side_1 * leftover;
    case 1:
        return start + side_1 + side_2 * leftover;
    case 2:
        return start + side_1 * (1 - leftover) + side_2;
    case 3:
        return start + side_2 * (1 - leftover);
    }
}
/**
 * @brief returns the position on a rounded square given time t
 * 
 * @param t time to get the point 
 * @return point2D 
 */
point2D roundedSquare(double t)
{
    double time_for_one_side = 10;
    double side_length = 1.5;
    double radius = 0.25 * side_length / 2;

    assert(radius<side_length);

    int count = int(t / time_for_one_side);
    double leftover = t - count * time_for_one_side;

    point2D start(0, 0);
    point2D side_1(1, 0);
    point2D side_2(0, 1);

    double mt = (side_length - 2 * radius) / (side_length - (2 - M_PI / 2) * radius) * time_for_one_side;
    double vel = ((side_length - (2 - M_PI / 2) * radius)) / time_for_one_side;

    if (leftover < mt)
    {
        /**
         * @brief straight line part
         * 
         */
        switch (count % 4)
        {
        case 0:
            return start + side_1 * leftover * vel - side_2 * radius;
        case 1:
            return start + side_1 * (side_length - radius) + side_2 * leftover * vel;
        case 2:
            return start + side_1 * (side_length - 2 * radius - leftover * vel) + side_2 * (side_length - radius);
        case 3:
            return start - side_1 * radius + side_2 * (side_length - 2 * radius - leftover * vel);
        }
    }
    else
    {
        /**
         * @brief Curved part
         * 
         */
        switch (count % 4)
        {
        case 0:
            return start + side_1 * (side_length - 2 * radius) + (side_2 * -radius).rotate(-(M_PI / 2) / (time_for_one_side - mt + EPS) *(leftover - mt));
        case 1:
            return start + (side_1 + side_2) * (side_length - 2 * radius) + (side_1 * radius).rotate(-(M_PI / 2) / (time_for_one_side - mt + EPS) *(leftover - mt));
        case 2:
            return start + side_2 * (side_length - 2 * radius) + (side_2 * radius).rotate(-(M_PI / 2) / (time_for_one_side - mt + EPS) *(leftover - mt));
        case 3:
            return start + (side_1 * -radius).rotate(-(M_PI / 2) / (time_for_one_side - mt + EPS) *(leftover - mt));
        }
    }
}