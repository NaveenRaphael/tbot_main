#include <eigen3/Eigen/Dense>
#include <array>
#include <ros/ros.h>
#include "tbot_main/velocityRequest.h"
#include "tbot_main/positionRequest.h"
#include <geometry_msgs/Twist.h>
#include "point.hpp"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

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
        position = func(0);
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

    double time;
    ros::Time time;

    Vector2d present_position;
    Vector2d x_solution;
    VectorXd l_solution;

    Vector2d cmd_vel;
    Vector2d x_solution_dot;
    VectorXd l_solution_dot;

    Vector2d destination;

    
    /**
     * @brief Construct a new Robot object
     * 
     * @param _name Name of the robot; needed for the tf
     * @param _rad radius of the robot
     * @param _alpha for how aggressively the robot moves to the end goal
     * @param _q See paper
     * @param _w See paper
     */
    Robot(std::string _name, double _rad = 0.25, double _alpha = 1, double _q = 1, double _w = 1.0 / N) : radius(_rad),
                                                                                                          num_obstacles(N),
                                                                                                          eye_2(Matrix2d::Identity()),
                                                                                                          alpha(_alpha),
                                                                                                          q(_q),
                                                                                                          w(_w),
                                                                                                          name(_name)
    {
        time = 0;

        l_solution = VectorXd(N);
        l_solution_dot = VectorXd(N);
        present_position.setZero();
        x_solution.setZero();
        l_solution.setZero();
        cmd_vel.setZero();
        x_solution_dot.setZero();
        l_solution_dot.setZero();

        destination << 4, 4;
    }
    /**
     * @brief The main method to integrate the dynamical system.
     * 
     * @param obs The list of obstacles with positions and velocities
     * @param present_time The present time; Not needed?
     */
    void loop(std::array<Obstacle, N> obs, double present_time)
    {
        double dt = present_time - time;
        time = present_time;
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

            solution_dot = Jinverse * (Hpred + HCorr) + Haug;

            x_solution_dot = solution_dot.head<2>();
            l_solution_dot = solution_dot.tail<N>();

            if (x_solution_dot.norm() > max_vel)
                scale = max_vel / x_solution_dot.norm();
            else
                scale = 1;
            x_solution_dot *= scale;

            x_solution += x_solution_dot / k * dt;
            l_solution += l_solution_dot / k * dt;
        }
        move(dt);
    }

    void move(double dt)
    {
        const double max_vel = 1.5;
        auto dis = x_solution - present_position;
        double scale;
        if (dis.norm() > max_vel)
            scale = max_vel / dis.norm();
        else
            scale = 1;
        cmd_vel = dis * scale;

        present_position += cmd_vel * dt;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tbot_dynamic_obstacle_avoidance");

    ros::Rate loop_rate(25);

    const int n = 5;
    int num = 100;
    std::array<Obstacle, n> obs = {
        Obstacle([](double t)
                 { return Vector2d(0, 1); }),
        Obstacle([](double t)
                 { return Vector2d(2, 1); }),
        Obstacle([](double t)
                 { return Vector2d(1, -2); }),
        Obstacle([](double t)
                 { return Vector2d(-1, 0); }),
        Obstacle([](double t)
                 { return Vector2d(-2, 0); })};
    Robot<n> rob("tf0");

    MatrixXd store = MatrixXd(num, 2);

    for (int i = 0; i < num; i++)
    {
        rob.loop(obs, i * 0.1);
        store.row(i) = rob.present_position;
    }
    std::cout << "Printing Values" << std::endl;
    std::cout << store;
    /**
     * @brief 
     * 
     *   

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
     * 
     */
}