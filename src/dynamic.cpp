#include <eigen3/Eigen/Dense>
#include <array>
#include <ros/ros.h>
#include "tbot_main/velocityRequest.h"
#include "tbot_main/positionRequest.h"
#include <geometry_msgs/Twist.h>
#include "point.hpp"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <iostream>

/**
 * @brief Things I need to do:
 *
 * 1. Proper ROS integration
 *      1. TF
 *      2. Publish to position control
 *
 */

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;

class Obstacle
{
public:
    Vector2d position;
    Vector2d velocity;
    Vector2d *function(double);
    const double radius;
    Obstacle(Vector2d func(double), double _rad = 0.3) : radius(_rad)
    {
        position = func(ros::Time::now().toSec());
        velocity.setZero();
    }
};
/**
 * @brief The Class to handle the Robot with dynamic obstacle avoidance
 *
 * @tparam N Number of obstacles has to be specified before.
 */
template <int N>
class Robot
{
public:
    const Matrix2d eye_2;
    const int num_obstacles;
    const double radius;
    const double alpha;
    const double q;
    const double w;
    const std::string name;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broad;
    tf::StampedTransform transform;
    geometry_msgs::Twist transform_dot;

    ros::ServiceClient position_change;

    // double time;
    ros::Time time;
    ros::Duration dt;
    ros::NodeHandle nh_;

    Vector2d present_position;
    Vector2d x_solution;
    VectorXd l_solution;

    Vector2d cmd_vel;
    Vector2d x_solution_dot;
    VectorXd l_solution_dot;

    Vector2d destination;

    int temporary;

    void get_current_location(bool = false);

    Robot(std::string, double = 0.25, double = 1.0, double = 1.0, double = 1.0 / N);

    void math_loop(std::array<Obstacle, N>);

    void loop(std::array<Obstacle, N>);

    void move(double);
    void publish();

    void new_EP();
    bool full_init;
};

/**
 * @brief Construct a new Robot object
 *
 * @param _name Name of the robot; needed for the tf
 * @param _rad radius of the robot
 * @param _alpha for how aggressively the robot moves to the end goal
 * @param _q See paper
 * @param _w See paper
 */
template <int N>
Robot<N>::Robot(std::string _name, double _rad, double _alpha, double _q, double _w) : radius(_rad),
                                                                                       num_obstacles(N),
                                                                                       eye_2(Matrix2d::Identity()),
                                                                                       alpha(_alpha),
                                                                                       q(_q),
                                                                                       w(_w),
                                                                                       name(_name),
                                                                                       nh_("~")
{
    time = ros::Time::now();
    l_solution = VectorXd(N);
    l_solution_dot = VectorXd(N);
    
    position_change = nh_.serviceClient<tbot_main::positionRequest>("/tb3_0/change_tbot_pos");

    full_init=false;
}
/**
 * @brief Gets and Sets new end position; WIP
 * 
 * @tparam N same as before
 */
template<int N>
void Robot<N>::new_EP(){
    get_current_location(true);
    x_solution = present_position;
    std::cout << present_position << std::endl;
    l_solution.setZero();
    cmd_vel.setZero();
    x_solution_dot.setZero();
    l_solution_dot.setZero();

    temporary=0;

    destination << 1, 0;

    ROS_INFO();

    full_init=true;
}
/**
 * @brief temporary function to simulate how the robot moves.
 *
 * @tparam N Number of obstacles
 * @param t differential time
 */
template <int N>
void Robot<N>::move(double t)
{
    const double max_vel = 1.5;
    auto dis = x_solution - present_position;
    double scale;
    if (dis.norm() > max_vel)
        scale = max_vel / dis.norm();
    else
        scale = 1;
    cmd_vel = dis * scale;

    present_position += cmd_vel * t;
}

/**
 * @brief Main method to solve differential equation for the obstacle avoidance
 *
 * @tparam N Number of obstacles
 * @param obs List of obstacles with their positions and velocities
 */
template <int N>
void Robot<N>::math_loop(std::array<Obstacle, N> obs)
{
    MatrixXd obstacle_positions(N, 2);
    MatrixXd obstacle_velocities(N, 2);
    VectorXd obstacle_radius(N);
    for (int i = 0; i < N; i++)
    {
        obstacle_positions.row(i) = obs.at(i).position.transpose();
        obstacle_velocities.row(i) = obs.at(i).velocity.transpose();
        obstacle_radius(i) = obs.at(i).radius;
    }
    MatrixXd A = obstacle_positions.rowwise() - present_position.transpose();
    MatrixXd Ap = obstacle_velocities.rowwise() - cmd_vel.transpose();
    VectorXd Anorm = (A.rowwise()).norm();
    VectorXd Anormi = Anorm.cwiseInverse();

    VectorXd Rd = ((obstacle_radius.array() - radius) * (obstacle_radius.array() + radius)).matrix();
    VectorXd Theta = 0.5 * (1 - Rd.cwiseProduct(Anormi.cwiseProduct(Anormi)).array()).matrix();
    VectorXd B = Theta.cwiseProduct(Anorm).cwiseProduct(Anorm) + A * present_position - radius * Anorm;
    // std::cout << A << "x-" << B << std::endl;
    VectorXd Alpha = (A * Ap.transpose()).diagonal(); // Check dimensions
    VectorXd Bp = Ap * present_position - radius * Alpha.cwiseProduct(Anormi) +
                  Rd.cwiseProduct(Alpha).cwiseProduct(Anormi).cwiseProduct(Anormi) + 2 * Theta.cwiseProduct(Alpha) +
                  A * cmd_vel;

    const int k = 10;

    Vector2d DerxL;
    DerxL.setZero();
    VectorXd G(N);
    MatrixXd Gd(N, N); // Check this
    VectorXd DertG;
    MatrixXd Mi(N, N);
    VectorXd Hpred(N + 2);
    VectorXd HCorr(N + 2);
    VectorXd Haug(N + 2);

    MatrixXd Jinverse(N + 2, N + 2);
    VectorXd solution_dot(N + 2);

    const double max_vel = 2;
    double scale;
    for (int i = 0; i < k; i++)
    {
        DerxL = (q * (x_solution - destination)) + w * A.transpose() * l_solution;
        G = w * (A * x_solution - B);
        Gd = G.asDiagonal();
        DertG = w * (Ap * x_solution - Bp + A * x_solution_dot);

        Mi = (Gd - w * w / q * (l_solution.asDiagonal() * A) * A.transpose()).inverse();

        Hpred.head<2>() = -w * Ap.transpose() * l_solution;
        Hpred.tail<N>() = -l_solution.cwiseProduct(DertG);

        HCorr.head<2>() = -alpha * DerxL;
        HCorr.tail<N>() = -Gd * l_solution;

        Haug.tail<N>() = -w * A * DerxL;
        Haug.head<2>() = w / q * A.transpose() * Haug.tail<N>();

        Jinverse.block<2, 2>(0, 0) = (eye_2 + w * w / q * A.transpose() * Mi * (l_solution.asDiagonal() * A)) / q;
        Jinverse.block<2, N>(0, 2) = -A.transpose() * Mi * w / q;
        Jinverse.block<N, 2>(2, 0) = -w / q * Mi * (l_solution.asDiagonal() * A);
        Jinverse.block<N, N>(2, 2) = Mi;

        if(temporary >0){
            std::cout<<Mi;
            temporary--;
        }

        solution_dot = Jinverse * (Hpred + HCorr) + Haug;

        x_solution_dot = solution_dot.head<2>();
        l_solution_dot = solution_dot.tail<N>();

        if (x_solution_dot.norm() > max_vel)
            scale = max_vel / x_solution_dot.norm();
        else
            scale = 1;
        x_solution_dot *= scale;

        x_solution += x_solution_dot / k * dt.toSec();
        l_solution += l_solution_dot / k * dt.toSec();
    }
}

/**
 * @brief The main loop, conatins all updates
 *
 * @tparam N same as before
 * @param obs List of obstacles and their positions and velocities
 */
template <int N>
void Robot<N>::loop(std::array<Obstacle, N> obs)
{
    if(!full_init){
        new_EP();
    }
    dt = ros::Time::now() - time;
    time = ros::Time::now();
    get_current_location();
    math_loop(obs);
    publish();
}
/**
 * @brief Get the current location and speed of the bot from ros TF
 *
 * @tparam N Same as every other place
 */
template <int N>
void Robot<N>::get_current_location(bool first)
{
    if (first)
    {
        try
        {
            sleep(1); //Without the sleep, the program crashes.

            tf_listener.lookupTransform(name, "world", ros::Time(0), transform);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            present_position<<0,0;
        }
    }
    tf_listener.lookupTransform( "world",name,  ros::Time(0), transform);
    present_position << transform.getOrigin().getX(), transform.getOrigin().getY();
    tf_listener.lookupTwist(name, "world",name,tf::Point(), "world", ros::Time(0), ros::Duration(0.2), transform_dot);

    // ROS_INFO("Present Position is apparently: (%f, %f)", present_position[0], present_position[1]);
    // double v = transform_dot.linear.x;
    // tf::Quaternion temp = transform.getRotation();
    // double ori = tf::getYaw(temp);
    // cmd_vel << v * cos(ori), v * sin(ori);
    cmd_vel<<transform_dot.linear.x, transform_dot.linear.y;
    // ROS_INFO("Vel: %f %f", cmd_vel[0], cmd_vel[1]);
}
/**
 * @brief publish next position the bot should move
 *
 * @tparam N same as before.
 */
template <int N>
void Robot<N>::publish()
{
    tbot_main::positionRequest val;
    val.request.x = x_solution[0];
    val.request.y = x_solution[1];
    // ROS_INFO("Waiting if service exists");
    position_change.waitForExistence(ros::Duration(3.0));
    if (position_change.call(val))
    {
        // ROS_INFO("Publishing: %f, %f", x_solution[0], x_solution[1]);
    }
    else
    {
        ROS_ERROR("Service does not exist~");
        ROS_ERROR("%s", position_change.getService().c_str());
    }

    tf::Transform temp;
    temp.setOrigin(tf::Vector3(x_solution[0],x_solution[1], 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    temp.setRotation(q);
    tf_broad.sendTransform(tf::StampedTransform(temp, ros::Time::now(), "world", "testing"));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tbot_dynamic_obstacle_avoidance");
    const int n = 2;
    Robot<n> rob("/tb0");

    int num = 100;
    std::array<Obstacle, n> obs = {
        Obstacle([](double t)
                 { return Vector2d(0, 1); }),
        Obstacle([](double t)
                 { return Vector2d(0.5, 0); }),
        // Obstacle([](double t)
        //          { return Vector2d(1, -2); }),
        // Obstacle([](double t)
        //          { return Vector2d(-1, 0); }),
        // Obstacle([](double t)
        //          { return Vector2d(-2, 0); })
        };

    ros::Rate loop_rate(25);

    while (ros::ok())
    {
        rob.loop(obs);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}