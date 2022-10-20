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

#ifndef NAV2_SMAC_PLANNER__CONSTANTS_HPP_
#define NAV2_SMAC_PLANNER__CONSTANTS_HPP_

#include <string>

namespace nav2_smac_planner
{
enum class MotionModel
{
  UNKNOWN = 0,
  TWOD = 1,
  DUBIN = 2,
  REEDS_SHEPP = 3,
  STATE_LATTICE = 4,
};

inline std::string toString(const MotionModel & n)
{
  switch (n) {
    case MotionModel::TWOD:
      return "2D";
    case MotionModel::DUBIN:
      return "Dubin";
    case MotionModel::REEDS_SHEPP:
      return "Reeds-Shepp";
    case MotionModel::STATE_LATTICE:
      return "State Lattice";
    default:
      return "Unknown";
  }
}

inline MotionModel fromString(const std::string & n)
{
  if (n == "2D") {
    return MotionModel::TWOD;
  } else if (n == "DUBIN") {
    return MotionModel::DUBIN;
  } else if (n == "REEDS_SHEPP") {
    return MotionModel::REEDS_SHEPP;
  } else if (n == "STATE_LATTICE") {
    return MotionModel::STATE_LATTICE;
  } else {
    return MotionModel::UNKNOWN;
  }
}

const float UNKNOWN = 255.0;
const float OCCUPIED = 254.0;
const float INSCRIBED = 253.0;
const float MAX_NON_OBSTACLE = 252.0;
const float FREE = 0;

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__CONSTANTS_HPP_
