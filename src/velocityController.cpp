#include <ros/ros.h>
#include "tbot_main/velocityRequest.h"
#include <geometry_msgs/Twist.h>

#define EPS 0.01

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define LINEAR_VELOCITY 0.3
#define ANGULAR_VELOCITY 1.5

class Velocity_drive
{
public:
    Velocity_drive();
    ~Velocity_drive();
    bool init();
    bool loop();
    void update_vel(float, float);
    ros::NodeHandle nh_;
    ros::ServiceServer change_vel;

    bool store_velocity(tbot_main::velocityRequest::Request &r,
                        tbot_main::velocityRequest::Response &g);

private:
    // ROS NodeHandle
    ros::NodeHandle nh_priv_;

    ros::Publisher cmd_vel_pub_;

    float vel;
    float omega;
};

Velocity_drive::Velocity_drive()
    : nh_priv_("~")
{
    //Init gazebo ros turtlebot3 node
    ROS_INFO("TurtleBot3 Velocity Node Init");
    auto ret = init();
    ROS_ASSERT(ret);
}

Velocity_drive::~Velocity_drive()
{
    vel = 0;
    omega = 0;
    ros::shutdown();
}

bool Velocity_drive::init()
{
    // initialize publishers
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    change_vel = nh_.advertiseService("change_tbot_vel", &Velocity_drive::store_velocity,this);

    return true;
}

bool Velocity_drive::loop()
{

    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = vel;
    cmd_vel.angular.z = omega;

    cmd_vel_pub_.publish(cmd_vel);
}

bool Velocity_drive::store_velocity(
    tbot_main::velocityRequest::Request &r,
    tbot_main::velocityRequest::Response &g)
{
    update_vel(r.v, r.w);

    g.r = true;

    return true;
}

void Velocity_drive::update_vel(float v, float w)
{
    vel = v;
    omega = w;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tbot_velocity");
    
    Velocity_drive tbot;

    ros::Rate loop_rate(125);
    while (ros::ok())
    {
        tbot.loop();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}