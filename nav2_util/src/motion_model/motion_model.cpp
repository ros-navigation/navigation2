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

#include "nav2_util/motion_model/motion_model.hpp"

#include <string>

namespace nav2_util
{

MotionModel *
MotionModel::createMotionModel(
  std::string & type, double alpha1, double alpha2,
  double alpha3, double alpha4, double alpha5)
{
  if (type == "differential") {
    return new DifferentialMotionModel(alpha1, alpha2, alpha3, alpha4);
  } else if (type == "omnidirectional") {
    return new OmniMotionModel(alpha1, alpha2, alpha3, alpha4, alpha5);
  }

  return nullptr;
}

}  // namespace nav2_util
