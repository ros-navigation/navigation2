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

#ifndef NAV2_LOCALIZATION__ANGLE_UTILS_HPP_
#define NAV2_LOCALIZATION__ANGLE_UTILS_HPP_

namespace nav2_localization
{
/**
 * @class AngleUtils
 * @brief Encapsulates useful methods to manipulate angles
 */
class AngleUtils
{
public:
    /**
     * @brief Calculates the difference between two angles and bounds the output to [-pi, pi]
     * @param a First angle
     * @param b Second angle
     * @return the difference between angle "a" and "b", bounded to [-pi, pi]
     */
    static double angleDiff(const double &a, const double &b);
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__ANGLE_UTILS_HPP_
