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

#ifndef NAV2_AMCL__MOTION_MODEL__DIFFERENTIAL_MOTION_MODEL_HPP_
#define NAV2_AMCL__MOTION_MODEL__DIFFERENTIAL_MOTION_MODEL_HPP_

#include <sys/types.h>
#include <math.h>
#include <algorithm>
#include "nav2_amcl/motion_model/motion_model.hpp"
#include "nav2_amcl/angleutils.hpp"


namespace nav2_amcl
{

class DifferentialMotionModel : public nav2_amcl::MotionModel
{
public:
  virtual void initialize(
    double alpha1, double alpha2, double alpha3, double alpha4,
    double alpha5);
  virtual void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta);

private:
  double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
};
}  // namespace nav2_amcl
#endif  // NAV2_AMCL__MOTION_MODEL__DIFFERENTIAL_MOTION_MODEL_HPP_
