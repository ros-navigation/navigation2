// Copyright (c) 2018 Intel Corporation
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_
#define NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_

#include <string>
#include <memory>

#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"

namespace nav2_amcl
{

/**
 * @class nav2_amcl::MotionModel
 * @brief An abstract motion model class
 */
class MotionModel
{
public:
  virtual ~MotionModel() = default;

  /**
   * @brief An factory to create motion models
   * @param type Type of motion model to create in factory
   * @param alpha1 error parameters, see documentation
   * @param alpha2 error parameters, see documentation
   * @param alpha3 error parameters, see documentation
   * @param alpha4 error parameters, see documentation
   * @param alpha5 error parameters, see documentation
   * @return MotionModel A pointer to the motion model it created
   */
  virtual void initialize(
    double alpha1, double alpha2, double alpha3, double alpha4,
    double alpha5) = 0;

  /**
   * @brief Update on new odometry data
   * @param pf The particle filter to update
   * @param pose pose of robot in odometry update
   * @param delta change in pose in odometry update
   */
  virtual void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta) = 0;
};
}  // namespace nav2_amcl

#endif  // NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_
