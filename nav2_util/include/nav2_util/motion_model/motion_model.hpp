// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_UTIL__MOTION_MODEL__MOTION_MODEL_HPP_
#define NAV2_UTIL__MOTION_MODEL__MOTION_MODEL_HPP_

#include <string>
#include "nav2_util/pf/pf.hpp"
#include "nav2_util/pf/pf_pdf.hpp"

namespace nav2_util
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

}  // namespace nav2_util

#endif  // NAV2_UTIL__MOTION_MODEL__MOTION_MODEL_HPP_
