// Copyright (c) 2026 Ocean Code AI Ltd
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

// NOTE: This file was authored with the assistance of an AI system,
// reviewed and validated by the submitter. Disclosed per the Nav2 PR
// template's AI-generated-software policy.

#ifndef NAV2_MONOCULAR_DEPTH_LAYER__DEPROJECTION_HPP_
#define NAV2_MONOCULAR_DEPTH_LAYER__DEPROJECTION_HPP_

#include <array>

namespace nav2_monocular_depth_layer
{

/**
 * @brief Deproject an image pixel with a known metric depth into a 3D point in
 *        the camera OPTICAL frame (REP 103: +x right, +y down, +z forward).
 *
 * @param u     pixel column
 * @param v     pixel row
 * @param depth metric depth along the optical axis, in metres (> 0)
 * @param fx    focal length x (pixels), CameraInfo K[0]
 * @param fy    focal length y (pixels), CameraInfo K[4]
 * @param cx    principal point x (pixels), CameraInfo K[2]
 * @param cy    principal point y (pixels), CameraInfo K[5]
 * @return {x, y, z} in the camera optical frame, in metres
 */
inline std::array<double, 3> deproject(
  double u, double v, double depth,
  double fx, double fy, double cx, double cy)
{
  return {
    (u - cx) * depth / fx,
    (v - cy) * depth / fy,
    depth
  };
}

}  // namespace nav2_monocular_depth_layer

#endif  // NAV2_MONOCULAR_DEPTH_LAYER__DEPROJECTION_HPP_
