#ifndef TBOT_POINT2D
#define TBOT_POINT2D

/**
 * @brief Class to handles points for my code. 
 * 
 * I should have ideally used normal class recommmended by ros, but I use this in too many places, so its too difficult to move away
 * 
 */
class point2D{
public:
    double x;
    double y;
    point2D(double, double);
    point2D();

    point2D operator -(const point2D&);
    point2D operator +(const point2D&);
    point2D operator *(const double&);

    double norm2();
    void set(double, double);
    point2D rotate(double);
    double get_angle();
};

point2D::point2D(double a, double b){
    x=a;
    y=b;
}

point2D::point2D(){
    x=0;
    y=0;
}

point2D point2D::operator-(const point2D& other){
    return point2D(this->x-other.x, this->y-other.y);
}

point2D point2D::operator+(const point2D& other){
    return point2D(this->x+other.x, this->y+other.y);
}

point2D point2D::operator*(const double& other){
    return point2D(this->x*other, this->y*other);
}

double point2D::norm2(){
    return x*x+y*y;
}

void point2D::set(double x, double y){
  x=x;
  y=y;
}

point2D point2D::rotate(double angle){
  double s= sin(angle), c=cos(angle);
  return point2D(x*c+y*s, -x*s+y*c);
}

double point2D::get_angle(){
  return atan2(y,x);
}

/**
 * @brief To find distance in the S2 Circle
 * 
 * @param toa From Angle
 * @param fra To angle
 * @return double the distance from angle1 to angle 2
 */
double s2dis(double toa, double fra)
{
  double dif = toa - fra;
  if (dif > M_PI)
  {
    dif -= 2 * M_PI;
  }
  else
  {
    if (dif < -M_PI)
    {
      dif += 2 * M_PI;
    }
  }
  return dif;
}

#endif