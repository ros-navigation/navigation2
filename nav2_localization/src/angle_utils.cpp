// Copyright (c) 2021 Khaled SAAD and Jose M. TORRES-CAMARA
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

#include <cmath>
#include "nav2_localization/angle_utils.hpp"

namespace nav2_localization
{
double AngleUtils::angleDiff(const double & a, const double & b)
{
  double angle = a - b;
  angle = fmod(angle, 2.0 * M_PI);  // Limit the angle from 0 to 2*Pi

  if (angle < M_PI && angle >= -M_PI) {  // Angle within the desired limit
    return angle;
  } else if (angle >= M_PI) {
    return angle - 2.0 * M_PI;
  } else {
    return angle + 2.0 * M_PI;
  }
}
}  // namespace nav2_localization
