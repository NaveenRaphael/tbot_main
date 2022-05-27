#ifndef TBOT_SOFTBODY
#define TBOT_SOFTBODY
#include "point.hpp"
#include <math.h>


class SoftBody2{
    /*
    Presently written for circular motion around center
    */
    int number;
    double start_time;

    point2D get_center(double);
    double get_orientation(double);
    point2D get_ithDir(int, double);
    double get_ithR(int, double);
    public:
    point2D get_ith(int, double);
    SoftBody2(int, double);
};

point2D SoftBody2::get_ith(int n, double time){
    return get_center(time)+get_ithDir(n, time)*get_ithR(n, time);
}

point2D SoftBody2::get_center(double time){
    return point2D(); //Center at origin
}

double SoftBody2::get_ithR(int n, double time){
    return 0.75;
}

double SoftBody2::get_orientation(double time){
    return (time-start_time)*0.15; 
}

point2D SoftBody2::get_ithDir(int n, double time){
    point2D unit =point2D(1,0);
    return unit.rotate(2.0*n/number*M_PI +get_orientation(time));
}

SoftBody2::SoftBody2(int n, double time){
    number=n;
    start_time=time;
}

#endif