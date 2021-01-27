// Copyright (c) 2021 Khaled SAAD and Jose M. TORRES-CAMARA

#include <cmath>
#include "nav2_localization/angle_utils.hpp"

namespace nav2_localization
{
double AngleUtils::angleDiff(const double &a, const double &b)
{
    double angle = a - b;
    angle = fmod(angle, 2.0*M_PI);  // Limit the angle from 0 to 2*Pi

    if(angle <= M_PI && angle >= -M_PI)  // Angle within the desired limit
        return angle;

    else if(angle > M_PI)
        return angle - 2.0*M_PI;

    else
        return angle + 2.0*M_PI;
}
}  // namespace nav2_localization
