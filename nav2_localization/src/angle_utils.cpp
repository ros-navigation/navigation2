#include "nav2_localization/angle_utils.hpp"
#include <cmath>

namespace nav2_localization
{
double AngleUtils::angle_diff(const double &a, const double &b)
{
    double angle = a - b;
    angle = fmod(angle, 2.0*M_PI); // limit the angle from 0 to 2*Pi

    if(angle <= M_PI && angle >= -M_PI) // angle within the desired limit
        return angle;

    else if(angle > M_PI)   
        return angle - 2.0*M_PI;
  
    else
        return angle + 2.0*M_PI;
}
}