#include <ros/ros.h>
#include "tbot_main/velocityRequest.h"
#include "tbot_main/trajectory.h"
#include <geometry_msgs/Twist.h>
#include "point.hpp"
#include "common_paths.hpp"
#include "softbody.hpp"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <functional>
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
                   { return circle_base(t, point2D(1, 1), 0.4, 2*M_PI / 30); }),
        trajectory(2, n, br, [](double t)
                   { return roundedSquare_base(t, point2D(0.3, 0.3), 12, 1.4, 0.4); })};

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
