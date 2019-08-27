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
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"

namespace nav2_amcl
{

class MotionModel
{
public:
  virtual ~MotionModel() = default;
  virtual void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta) = 0;

  static MotionModel * createMotionModel(
    std::string & type, double alpha1, double alpha2,
    double alpha3, double alpha4, double alpha5);
};

class OmniMotionModel : public MotionModel
{
public:
  OmniMotionModel(double alpha1, double alpha2, double alpha3, double alpha4, double alpha5);
  void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta);

private:
  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
  double alpha5_;
};

class DifferentialMotionModel : public MotionModel
{
public:
  DifferentialMotionModel(double alpha1, double alpha2, double alpha3, double alpha4);
  void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta);

private:
  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
};

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_
