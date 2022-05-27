#include <iostream>
#include <eigen3/Eigen/Dense>
#include <array>
#include <vector>
#include <memory>

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;

class A
{
public:
    int a;
    int b;

    A(int a = 1, int b = 2) : a(a), b(b)
    {
        std::cout << "Constructing A" << std::endl;
    }

    virtual void loop()
    {
        std::cout << "Looping through A" << std::endl;
    }
};

class B : public A
{
public:
    int c;

    B(int a = 1, int b = 2, int c = 3) : A(a, b), c(c)
    {
        std::cout << "Constructing B" << std::endl;
    }

    void loop() override
    {
        std::cout << "Looping through B" << std::endl;
    }
};

class C : public A
{
public:
    int c;

    C(int a = 1, int b = 2, int c = 3) : A(a, b), c(c)
    {
        std::cout << "Constructing C" << std::endl;
    }

    void loop() override
    {
        std::cout << "Looping through C" << std::endl;
    }
};

class Obstacle
{
public:
    Vector2d position;
    Vector2d velocity;
    const double radius;
    Obstacle(Vector2d func(double), double _rad = 0.3) : radius(_rad)
    {
        position = func(0);
        velocity.setZero();
    };
};

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

    double time;

    Vector2d present_position;
    Vector2d x_solution;
    VectorXd l_solution;

    Vector2d cmd_vel;
    Vector2d x_solution_dot;
    VectorXd l_solution_dot;

    Vector2d destination;

    Robot(double _rad = 0.25, double _alpha = 5, double _q = 1, double _w = 1.0 / N) : radius(_rad),
                                                                                       num_obstacles(N),
                                                                                       eye_2(Matrix2d::Identity()),
                                                                                       alpha(_alpha),
                                                                                       q(_q),
                                                                                       w(_w)
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

        destination << 2, 2.2;
    }
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
        /*std::cout << A(1,1) << "x+"<<A(2,1)<<"y-" << B << std::endl;*/
        VectorXd Alpha = (A * Ap.transpose()).diagonal(); // Check dimensions
        VectorXd Bp = Ap * present_position - radius * Alpha.cwiseProduct(Anormi) +
                      Rd.cwiseProduct(Alpha).cwiseProduct(Anormi).cwiseProduct(Anormi) + 2 * Theta.cwiseProduct(Alpha) +
                      A * (cmd_vel);

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

            l_solution = l_solution.cwiseMax(0);
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

template<int N>
void tempo(std::array<std::unique_ptr<A>, N> &ar){
    for(const auto& a:ar){
        a->loop();
    }
}

int main()
{
    /**
    // const int n = 4;
    // int num = 200;
    // std::array<Obstacle, n> obs = {
    //     Obstacle([](double t)
    //              { return Vector2d(1, 1); },
    //              0.2),
    //     Obstacle([](double t)
    //              { return Vector2d(0, 1); },
    //              0.1),
    //     Obstacle([](double t)
    //              { return Vector2d(2, 1); },
    //              0.1),
    //     Obstacle([](double t)
    //              { return Vector2d(1, 2); },
    //              0.1),
    //     // Obstacle([](double t) {return Vector2d(-2, 0); })*/
    // };
    // Robot<obs.size()> rob(0.25, 1, 1, 1);

    // MatrixXd store = MatrixXd(num, 2);

    // for (int i = 0; i < num; i++)
    // {
    //     rob.loop(obs, i * 0.1);
    //     store.row(i) = rob.present_position;
    // }
    // std::cout << "Printing Values" << std::endl;
    // for (int i = 0; i < num; i++)
    // {
    //     std::cout << "(" << store.row(i)[0] << ", " << store.row(i)[1] << ")\n";
    // }

    // auto temp1 = MatrixXd(2, 4);
    // temp1 <<
    //  1, 2, 4, 6,
    //  2, 3, 6, 7;
    // auto temp2 = VectorXd(2);
    // temp2 << 3, 4;
    // std::cout << temp2.asDiagonal()*temp1;

    std::array<std::unique_ptr<A>, 2> ar
    {
        std::make_unique<B>(B(2, 3, 4)),
        std::make_unique<C>(C(3, 4, 5))
    };

    for(auto& a: ar){
        a->loop();
    }

    tempo<ar.size()>(ar);
}