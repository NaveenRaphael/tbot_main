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
#include <string>
#include <vector>
#include <memory>

/**
 * @brief Implements Mrs Rejitha's dynamical approach for obstacle avoidance
 *
 * @todo Fix the bug that rosnamespace does not work for this
 *
 * This looks like completed code, if I say so myself
 *
 */

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;

/**
 * @brief A virtual class for the types of obstacles
 *
 * There are Simulated obstacles, and actual robot obstacles we can get the positions using tf
 *
 */
class Obstacle
{
public:
    std::string name;
    Vector2d position;
    Vector2d velocity;
    const double radius;

    virtual void loop(){};

    Obstacle(double r, std::string n) : radius(r)
    {
        name = n;
    }
};

/**
 * @brief The derived class for the simulated obstacle.
 *
 * Presently generates a fixed obstacle at a location.
 * Supposed to make a simulated moving obstacle
 * Also publishes the position of the simulated obstacle in tf
 *
 * @todo Presently only fixed obstacles can be made. Fix the lambda function
 */
class Virtual_Obstacle : public Obstacle
{
public:
    tf::TransformBroadcaster tf_broad;
    /**
     * @brief Construct a new Virtual_Obstacle object
     *
     * @param n Name of the obstacle as shown in tf
     * @param func function to model the trajectory of the robot
     * @param _rad radius of the robot
     */
    Virtual_Obstacle(std::string n, Vector2d func(double), double _rad = 0.2) : Obstacle(_rad, n)
    {
        position = func(ros::Time::now().toSec());
        velocity.setZero();
    }
    void tf_publish();
    /**
     * @brief To be put in ros::ok() loop
     *
     */
    void loop()
    {
        tf_publish();
    }
};

/**
 * @brief Publish the present position of the obstacle in tf
 *
 */
void Virtual_Obstacle::tf_publish()
{
    tf::Transform temp;
    temp.setOrigin(tf::Vector3(position[0], position[1], 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    temp.setRotation(q);
    tf_broad.sendTransform(tf::StampedTransform(temp, ros::Time::now(), "world", name));
}

/**
 * @brief Derived from Virtual_Obstacle, essentially, the stores the locations and velocities of robot obstacles from tf
 *
 */
class Robot_Obstacle : public Obstacle
{
public:
    bool full_init;
    tf::TransformListener &tf_listener;
    geometry_msgs::Twist transform_dot;
    tf::StampedTransform transform;

    /**
     * @brief to be put in ros::ok() loop
     *
     */
    void loop()
    {
        if (!full_init)
        {
            // This might not be necessary
            sleep(1);
            full_init = true;
        }
        tf_get();
    }

    void tf_get();
    /**
     * @brief Construct a new Robot_Obstacle object
     *
     * @param n Name of the robot as seen in tf
     * @param p transform listener
     */
    Robot_Obstacle(std::string n, tf::TransformListener &p) : Obstacle(0.15, n), transform_dot(), transform(), tf_listener(p)
    {
        full_init = false;
        position << 0, 0;
        velocity.setZero();
    }
};

/**
 * @brief Get the position of the obstacles from tf
 *
 */
void Robot_Obstacle::tf_get()
{

    tf_listener.lookupTransform("world", name, ros::Time(0), transform);
    position << transform.getOrigin().getX(), transform.getOrigin().getY();

    tf_listener.lookupTwist(name, "world", name, tf::Point(), "world", ros::Time(0), ros::Duration(0.2), transform_dot);
    velocity << transform_dot.linear.x, transform_dot.linear.y;
}

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
    tf::TransformListener &tf_listener;
    tf::TransformBroadcaster tf_broad;
    tf::StampedTransform transform;
    geometry_msgs::Twist transform_dot;

    ros::ServiceClient position_change;

    ros::ServiceServer change_EP;
    bool store_position(tbot_main::positionRequest::Request &r,
                        tbot_main::positionRequest::Response &g);
    bool store_position(double, double);

    // double time;
    ros::Time time;
    ros::Duration dt;
    ros::NodeHandle &nh_;

    Vector2d present_position;
    Vector2d x_solution;
    VectorXd l_solution;

    Vector2d cmd_vel;
    Vector2d x_solution_dot;
    VectorXd l_solution_dot;

    Vector2d destination;

    int temporary;

    void get_current_location(bool = false);

    Robot(std::string, ros::NodeHandle &, tf::TransformListener &, double = 0.1, double = 1.0, double = 1.0, double = 1.0 / N);

    void math_loop(std::array<std::unique_ptr<Obstacle>, N> &);

    void loop(std::array<std::unique_ptr<Obstacle>, N> &);

    void move(double);
    void publish();

    void init_start_pos();
    bool full_init;

    void set_zero();
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
Robot<N>::Robot(std::string _name, ros::NodeHandle &nh, tf::TransformListener &P, double _rad, double _alpha, double _q, double _w) : radius(_rad),
                                                                                                                                      num_obstacles(N),
                                                                                                                                      eye_2(Matrix2d::Identity()),
                                                                                                                                      alpha(_alpha),
                                                                                                                                      q(_q),
                                                                                                                                      w(_w),
                                                                                                                                      name(_name),
                                                                                                                                      nh_(nh),
                                                                                                                                      tf_listener(P)
{
    l_solution = VectorXd(N);
    l_solution_dot = VectorXd(N);

    position_change = nh_.serviceClient<tbot_main::positionRequest>("change_tbot_pos");

    change_EP = nh_.advertiseService("OA_change_tbot_pos",
                                     &Robot<N>::store_position, this);

    full_init = false;
}

/**
 * @brief Setting values 0
 *
 * @tparam N
 */
template <int N>
void Robot<N>::set_zero()
{
    cmd_vel.setZero();
    x_solution_dot.setZero();
    l_solution_dot.setZero();
}

/**
 * @brief Sets new end position for the robot
 *
 * @tparam N same as always
 * @param r New final position
 * @param g response
 * @return true required by ros
 * @return false
 */
template <int N>
bool Robot<N>::store_position(tbot_main::positionRequest::Request &r,
                              tbot_main::positionRequest::Response &g)
{

    set_zero();

    destination << r.x, r.y;
    g.r = true;
    return true;
}

/**
 * @brief Sets end position in this code
 *
 * @tparam N same as before
 * @param x
 * @param y
 * @return true required by ros
 * @return false
 */
template <int N>
bool Robot<N>::store_position(double x, double y)
{

    set_zero();
    destination << x, y;
    return true;
}
/**
 * @brief Gets and Sets new end position; WIP
 *
 * @tparam N same as before
 */
template <int N>
void Robot<N>::init_start_pos()
{
    get_current_location(true);
    x_solution = present_position;
    l_solution.setZero();

    temporary = 0;

    ROS_INFO("Initialising positions: \n Initial position: (%f, %f), \nX_sol: (%f, %f)", present_position[0], present_position[1], x_solution[0], x_solution[1]);

    full_init = true;
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
void Robot<N>::math_loop(std::array<std::unique_ptr<Obstacle>, N> &obs)
{
    MatrixXd obstacle_positions(N, 2);
    MatrixXd obstacle_velocities(N, 2);
    VectorXd obstacle_radius(N);
    for (int i = 0; i < N; i++)
    {
        obstacle_positions.row(i) = obs.at(i)->position.transpose();
        obstacle_velocities.row(i) = obs.at(i)->velocity.transpose();
        obstacle_radius(i) = obs.at(i)->radius;
    }
    MatrixXd A = obstacle_positions.rowwise() - present_position.transpose();
    MatrixXd Ap = obstacle_velocities.rowwise() - cmd_vel.transpose();

    VectorXd Anorm = (A.rowwise()).norm();
    VectorXd Anormi = Anorm.cwiseInverse();

    VectorXd Rd = ((obstacle_radius.array() - radius) * (obstacle_radius.array() + radius)).matrix();
    VectorXd Theta = 0.5 * (1 - Rd.cwiseProduct(Anormi.cwiseProduct(Anormi)).array()).matrix();
    VectorXd B = Theta.cwiseProduct(Anorm).cwiseProduct(Anorm) + A * present_position - radius * Anorm;
    VectorXd Alpha = (A * Ap.transpose()).diagonal(); // Check dimensions
    VectorXd Bp = Ap * present_position - radius * Alpha.cwiseProduct(Anormi) +
                  Rd.cwiseProduct(Alpha).cwiseProduct(Anormi).cwiseProduct(Anormi) + 2 * Theta.cwiseProduct(Alpha) + A * cmd_vel;

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

    const double max_vel = 0.7;
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

        l_solution = l_solution.cwiseMax(0);
    }
}

/**
 * @brief The main loop, conatins all updates
 *
 * @tparam N same as before
 * @param obs List of obstacles and their positions and velocities (pass by reference)
 */
template <int N>
void Robot<N>::loop(std::array<std::unique_ptr<Obstacle>, N> &obs)
{
    if (!full_init)
    {
        init_start_pos();
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
            sleep(1); // Without the sleep, the program crashes.
            time = ros::Time::now();
            tf_listener.lookupTransform(name, "world", ros::Time(0), transform);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            present_position << 0, 0;
        }
    }

    tf_listener.lookupTransform("world", name, ros::Time(0), transform);
    present_position << transform.getOrigin().getX(), transform.getOrigin().getY();
    tf_listener.lookupTwist(name, "world", name, tf::Point(), "world", ros::Time(0), ros::Duration(0.2), transform_dot);

    cmd_vel << transform_dot.linear.x, transform_dot.linear.y;
}

/**
 * @brief publish next position the bot should move
 *
 * @tparam N same as before.
 */
template <int N>
void Robot<N>::publish()
{
    /**
     * @brief Publishing the Position to go to
     *
     */
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

    /**
     * @brief publishing the same point in tf
     *
     */
    tf::Transform temp;
    temp.setOrigin(tf::Vector3(x_solution[0], x_solution[1], 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    temp.setRotation(q);
    tf_broad.sendTransform(tf::StampedTransform(temp, ros::Time::now(), "world", "testing"));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tbot_dynamic_obstacle_avoidance");
    ros::NodeHandle nh("~");
    tf::TransformListener tf_listener;
    const int n = 2;
    Robot<n> rob("/tb0", nh, tf_listener);

    /**
     * @brief Array with pointers to different types of obstacles; Needs to be done this way because of type safety of C++
     *
     */
    std::array<std::unique_ptr<Obstacle>, n> obs{
        // std::make_unique<Robot_Obstacle>(Robot_Obstacle("tb1", tf_listener)),
        // std::make_unique<Robot_Obstacle>(Robot_Obstacle("tb2")),
        std::make_unique<Virtual_Obstacle>(
            Virtual_Obstacle(
                "Virtual_1", [](double t)
                { return Vector2d(2, 1); },
                0.1)),
        std::make_unique<Virtual_Obstacle>(
            Virtual_Obstacle(
                "Virtual_2", [](double t)
                { return Vector2d(1, 2); },
                0.1)),
        // // std::make_unique<Virtual_Obstacle>(Virtual_Obstacle(
        // "Virtual_3", [](double t)
        // { return Vector2d(1, 2); },
        // 0.1))
    };

    ros::Rate loop_rate(25);

    while (ros::ok())
    {
        for (auto &ob : obs)
        {
            ob->loop();
        }
        rob.loop(obs);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
