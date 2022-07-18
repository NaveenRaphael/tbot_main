#include "point.hpp"
#include <math.h>

#define EPS 0.01

/**
 * Some basic trajectories:
 *Circle
[](double t){
double phase=omega *t;
return point2D(center.x, center.y)+ point2D(sin(phase), cos(phase))*radius;}
 *
 */

/**
 * @brief Base for generating circle
 *
 * @param t time
 * @param center center of circle
 * @param r radius of circle
 * @param omega rotational speed
 * @return point2D point the robot is expected
 */
point2D circle_base(double t, point2D center, double r, double omega)
{
    return center + point2D(r, 0).rotate(omega * t);
}

/**
 * @brief Deprecated
 *
 * @param t
 * @return point2D
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
 * @brief 
 * 
 * @param t time parameter
 * @param start position of first corner of _inner_ square
 * @param time_for_one_side time taken to travel one side
 * @param side_length side length of the square, ignoring the rounded edge
 * @param p Percent of have side for rounding
 * @return point2D point the robot is supposed to be at this time. 
 */
point2D roundedSquare_base(double t, point2D fstart, double time_for_one_side, double side_length, double p)
{
    // double time_for_one_side = 10;
    // double side_length = 1;
    double radius = p * side_length / 2;

    point2D start(fstart.x+radius, fstart.y+radius);

    int count = int(t / time_for_one_side);
    double leftover = t - count * time_for_one_side;

    // point2D start(0.2, 0.2);
    point2D side_1(1, 0); // Directions
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
            return start + side_1 * (side_length - 2 * radius) + (side_2 * -radius).rotate(-(M_PI / 2) / (time_for_one_side - mt + EPS) * (leftover - mt));
        case 1:
            return start + (side_1 + side_2) * (side_length - 2 * radius) + (side_1 * radius).rotate(-(M_PI / 2) / (time_for_one_side - mt + EPS) * (leftover - mt));
        case 2:
            return start + side_2 * (side_length - 2 * radius) + (side_2 * radius).rotate(-(M_PI / 2) / (time_for_one_side - mt + EPS) * (leftover - mt));
        case 3:
            return start + (side_1 * -radius).rotate(-(M_PI / 2) / (time_for_one_side - mt + EPS) * (leftover - mt));
        }
    }
}
