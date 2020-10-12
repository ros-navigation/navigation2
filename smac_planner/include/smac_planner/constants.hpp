// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef SMAC_PLANNER__CONSTANTS_HPP_
#define SMAC_PLANNER__CONSTANTS_HPP_

#include <string>

namespace smac_planner
{
enum class MotionModel
{
  UNKNOWN = 0,
  VON_NEUMANN = 1,
  MOORE = 2,
  DUBIN = 3,
  REEDS_SHEPP = 4,
};

inline std::string toString(const MotionModel & n)
{
  switch (n) {
    case MotionModel::VON_NEUMANN:
      return "Von Neumann";
    case MotionModel::MOORE:
      return "Moore";
    case MotionModel::DUBIN:
      return "Dubin";
    case MotionModel::REEDS_SHEPP:
      return "Reeds-Shepp";
    default:
      return "Unknown";
  }
}

inline MotionModel fromString(const std::string & n)
{
  if (n == "VON_NEUMANN") {
    return MotionModel::VON_NEUMANN;
  } else if (n == "MOORE") {
    return MotionModel::MOORE;
  } else if (n == "DUBIN") {
    return MotionModel::DUBIN;
  } else if (n == "REEDS_SHEPP") {
    return MotionModel::REEDS_SHEPP;
  } else {
    return MotionModel::UNKNOWN;
  }
}

const float UNKNOWN = 255;
const float OCCUPIED = 254;
const float INSCRIBED = 253;
const float MAX_NON_OBSTACLE = 252;
const float FREE = 0;

}  // namespace smac_planner

#endif  // SMAC_PLANNER__CONSTANTS_HPP_
