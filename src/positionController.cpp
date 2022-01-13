#include <ros/ros.h>
#include "tbot_main/velocityRequest.h"
#include "tbot_main/positionRequest.h"
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

/*
To Do:

Position Drive: listens to end position; current position; generates command velocity

*/

class Position_drive
{
public:
    Position_drive();
    ~Position_drive();
    bool init(double, double);
    bool loop();                               //Run every iteration
    void update_fpos(double, double);          //Update end position
    void update_inpos(double, double, double); //Update init position
    void update_vel();                         //Generate velocities

    ros::NodeHandle nh_;
    ros::ServiceServer change_pos;
    ros::Subscriber get_odom;

    bool store_position(tbot_main::positionRequest::Request &r,
                        tbot_main::positionRequest::Response &g);

    void update_ipos(const nav_msgs::Odometry &data);
    tf::Transform get_transform();
    std::string get_TF_name();

private:
    // ROS NodeHandle
    ros::NodeHandle nh_priv_;

    ros::ServiceClient cmd_vel_pub_;

    point2D start;
    point2D present;
    double ori;
    point2D fin;

    std::string TF_name;

    bool lazy;

    double v;
    double om;
};

Position_drive::Position_drive()
    : nh_priv_("~")
{
    //Init gazebo ros turtlebot3 node
    double x, y;
    nh_.getParam("x", x);
    nh_.getParam("y", y);
    nh_.getParam("tf", TF_name);
    ROS_INFO("TurtleBot3 Position Node Init");
    // ROS_INFO(x)
    auto ret = init(x, y);
    ROS_ASSERT(ret);
    lazy = true;
}

Position_drive::~Position_drive()
{
    ros::shutdown();
}

void Position_drive::update_ipos(const nav_msgs::Odometry &data)
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

bool Position_drive::init(double x, double y)
{
    // initialize publishers
    start = point2D(-x, -y);
    ROS_INFO("Start position: %f, %f", start.x, start.y);
    cmd_vel_pub_ = nh_.serviceClient<tbot_main::velocityRequest>("change_tbot_vel");
    change_pos = nh_.advertiseService("change_tbot_pos", &Position_drive::store_position, this);
    get_odom = nh_.subscribe("odom", 1000, &Position_drive::update_ipos, this);

    return true;
}

bool Position_drive::loop()
{
    if (!lazy)
    {
        tbot_main::velocityRequest val;
        update_vel();

        val.request.v = v;
        val.request.w = om;

        cmd_vel_pub_.call(val);
    }
}

bool Position_drive::store_position(
    tbot_main::positionRequest::Request &r,
    tbot_main::positionRequest::Response &g)
{
    update_fpos(r.x, r.y);
    g.r = true;
    return true;
}

void Position_drive::update_fpos(double x, double y)
{
    fin.x = x;
    fin.y = y;
    lazy = false;
}
/*
Control Law:
Given present position, and final position, to find v and omega
*/
void Position_drive::update_vel()
{
    point2D error = fin - present;
    if (error.norm2() < EPS)
    {
        lazy = true;
        v=0;
        om=0;
        return;
    }
    double ang = atan2(error.y, error.x);

    double eror = s2dis(ang, ori);
    double ernorm = error.norm2();

    v = ernorm * cos(eror);
    om = eror;
}

tf::Transform Position_drive::get_transform()
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(present.x, present.y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, ori);
    transform.setRotation(q);
    return transform;
}

std::string Position_drive::get_TF_name()
{
    return TF_name;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tbot_position");

    Position_drive tbot;
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