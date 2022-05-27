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

#define Pv 5
#define Pw 5

/**
 * @brief Gets end position via a service, and generates velocity which is is sent via another service
 * 
 */
class Position_drive
{
public:
    Position_drive();
    ~Position_drive();
    void loop();                               //Run every iteratio
    void update_fpos(double, double);          //Update end position
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

/**
 * @brief Construct a new Position_drive::Position_drive object
 * 
 * First starts the node_handle, and gets the rosparams
 * Initialises other variables and ros functions
 * 
 */
Position_drive::Position_drive()
    : nh_priv_("~")
{
    double x, y;
    nh_.getParam("x", x);
    nh_.getParam("y", y);
    nh_.getParam("tf", TF_name);
    ROS_INFO("TurtleBot3 Position Node Init");

    start = point2D(-x, -y);
    ROS_INFO("Start position: %f, %f", start.x, start.y);

    cmd_vel_pub_ = nh_.serviceClient<tbot_main::velocityRequest>("change_tbot_vel");
    change_pos = nh_.advertiseService("change_tbot_pos", &Position_drive::store_position, this);
    
    get_odom = nh_.subscribe("odom", 1000, &Position_drive::update_ipos, this);

    lazy = true;
}
/**
 * @brief Destroy the Position_drive::Position_drive object
 * 
 */
Position_drive::~Position_drive()
{
    ros::shutdown();
}

/**
 * @brief Gets the odometry data from the robot and updates the position
 * 
 * @param data Odometry data from the robot
 */
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

/**
 * @brief Function to call inside ros::ok()
 * 
 */
void Position_drive::loop()
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

/**
 * @brief Function for the service, updates the new end position
 * 
 * @param r request: The position the robot is supposed to go
 * @param g response: True, 
 * @return true Always returns true, needed by ros
 * @return false 
 */
bool Position_drive::store_position(
    tbot_main::positionRequest::Request &r,
    tbot_main::positionRequest::Response &g)
{
    fin.x = r.x;
    fin.y = r.y;
    lazy = false;

    g.r = true;
    return true;
}


/**
 * @brief Given the endpoint is not close enough to the present position of the robot, updates the command velocities.
 * 
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

    v = Pv*ernorm * cos(eror);
    om = Pw*eror;
}

/**
 * @brief Gets the tf transfrom of this robot
 * 
 * @return tf::Transform 
 */
tf::Transform Position_drive::get_transform()
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(present.x, present.y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, ori);
    transform.setRotation(q);
    return transform;
}

/**
 * @brief Returns the name of the tf frame
 * 
 * @return std::string 
 */
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