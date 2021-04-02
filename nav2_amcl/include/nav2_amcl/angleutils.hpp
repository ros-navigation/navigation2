/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef NAV2_AMCL__ANGLEUTILS_HPP_
#define NAV2_AMCL__ANGLEUTILS_HPP_

#include <math.h>

namespace nav2_amcl
{

/*
 * @class angleutils
 * @brief Some utilities for working with angles
 */
class angleutils
{
public:
  /*
   * @brief Normalize angles
   * @brief z Angle to normalize
   * @return normalized angle
   */
  static double normalize(double z);

  /*
   * @brief Find minimum distance between 2 angles
   * @brief a Angle 1
   * @brief b Angle 2
   * @return normalized angle difference
   */
  static double angle_diff(double a, double b);
};

inline double
angleutils::normalize(double z)
{
  return atan2(sin(z), cos(z));
}

inline double
angleutils::angle_diff(double a, double b)
{
  a = normalize(a);
  b = normalize(b);
  double d1 = a - b;
  double d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0) {
    d2 *= -1.0;
  }
  if (fabs(d1) < fabs(d2)) {
    return d1;
  } else {
    return d2;
  }
}

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__ANGLEUTILS_HPP_
