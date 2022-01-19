/*
TODO:
implement the control law in https://www.sciencedirect.com/science/article/pii/S2405896315013890

needs the following:

Need to think:
How to select different trajectories?

*/

#include <ros/ros.h>
#include "tbot_main/velocityRequest.h"
#include "tbot_main/trajectory.h"
#include <geometry_msgs/Twist.h>
#include "point.hpp"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#define EPS 0.01

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define LINEAR_VELOCITY 0.3
#define ANGULAR_VELOCITY 1.5

#define c1 1
#define c2 0.2
#define c3 1

class Trajectory_control
{
public:
    Trajectory_control();
    ~Trajectory_control();
    bool init(double, double);
    void loop(); //Run every iteration

    void update_vel(); //Generate velocities

    ros::NodeHandle nh_;

    ros::Subscriber trajectory;
    ros::Subscriber get_odom;

    void get_trajectory(const tbot_main::trajectory &data); //New
    void update_ipos(const nav_msgs::Odometry &data);
    tf::Transform get_transform();
    std::string get_TF_name();

private:
    // ROS NodeHandle
    ros::NodeHandle nh_priv_;

    ros::ServiceClient cmd_vel_pub_;

    tbot_main::trajectory togo;

    point2D start;
    point2D present;
    double ori;

    std::string TF_name;

    double v;
    double om;
};

Trajectory_control::Trajectory_control()
    : nh_priv_("~")
{
    //Init gazebo ros turtlebot3 node
    double x, y;
    nh_.getParam("x", x);
    nh_.getParam("y", y);
    nh_.getParam("tf", TF_name);
    ROS_INFO("TurtleBot3 Trajectory Control Node Init");
    // ROS_INFO(x)
    auto ret = init(x, y);
    ROS_ASSERT(ret);
}

Trajectory_control::~Trajectory_control()
{
    ros::shutdown();
}

void Trajectory_control::update_ipos(const nav_msgs::Odometry &data)
{
    double x = data.pose.pose.position.x;
    double y = data.pose.pose.position.y;
    point2D temp;
    geometry_msgs::Quaternion ori2 = data.pose.pose.orientation;

    double siny = 2.0 * (ori2.w * ori2.z + ori2.x * ori2.y);
    double cosy = 1.0 - 2.0 * (ori2.y * ori2.y + ori2.z * ori2.z);
    double ang = atan2(siny, cosy);

    temp.x = x;
    temp.y = y;

    present = temp - start;
    ori = ang;
}

void Trajectory_control::get_trajectory(const tbot_main::trajectory &data)
{
    togo = data;
}

bool Trajectory_control::init(double x, double y)
{
    // initialize publishers
    start = point2D(-x, -y);
    ROS_INFO("Start position: %f, %f", start.x, start.y);
    cmd_vel_pub_ = nh_.serviceClient<tbot_main::velocityRequest>("change_tbot_vel");

    trajectory = nh_.subscribe("trajectory", 100, &Trajectory_control::get_trajectory, this);
    get_odom = nh_.subscribe("odom", 1000, &Trajectory_control::update_ipos, this);

    return true;
}

void Trajectory_control::loop()
{
    tbot_main::velocityRequest val;
    update_vel();

    val.request.v = v;
    val.request.w = om;

    cmd_vel_pub_.call(val);
}

void Trajectory_control::update_vel()
{
    point2D traj = point2D(togo.x, togo.y);
    point2D errorP = (traj-present).rotate(ori);
    double errorT = s2dis(togo.t,ori);
    double denom=sqrt(1+errorP.norm2());
    v= togo.v+ c1*errorP.x/denom;
    om=togo.w + c2*togo.v*(errorP.y*cos(errorT/2)-errorP.x*sin(errorT/2))/denom+ c3*sin(errorT/2);
    
    // ROS_INFO("v:%.3f, w: %.3f, error_x: %.3f, error_y: %.3f, error_orient: %.3f", v, om, errorP.x, errorP.y, errorT*RAD2DEG);

}

tf::Transform Trajectory_control::get_transform()
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(present.x, present.y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, ori);
    transform.setRotation(q);
    return transform;
}

std::string Trajectory_control::get_TF_name()
{
    return TF_name;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tbot_position");

    Trajectory_control tbot;
    static tf::TransformBroadcaster br;
    std::string base_name = tbot.get_TF_name();

    ros::Rate loop_rate(125);
    while (ros::ok())
    {
        tbot.loop();
        br.sendTransform(tf::StampedTransform(tbot.get_transform(), ros::Time::now(), "world", base_name));

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}