/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__PARTICLE_CLOUD_DISPLAY_HPP_
#define NAV2_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__PARTICLE_CLOUD_DISPLAY_HPP_

#include <memory>
#include <vector>

#include "nav2_msgs/msg/particle_cloud.hpp"

#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/message_filter_display.hpp"

namespace Ogre
{
class ManualObject;
}  // namespace Ogre

namespace rviz_common
{
namespace properties
{
class EnumProperty;
class ColorProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_rendering
{
class Arrow;
class Axes;
}  // namespace rviz_rendering

namespace nav2_rviz_plugins
{
class FlatWeightedArrowsArray;
struct OgrePoseWithWeight
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  float weight;
};

/** @brief Displays a nav2_msgs/ParticleCloud message as a bunch of line-drawn weighted arrows. */
class ParticleCloudDisplay : public rviz_common::MessageFilterDisplay<nav2_msgs::msg::ParticleCloud>
{
  Q_OBJECT

public:
  // TODO(botteroa-si): Constructor for testing, remove once ros_nodes can be mocked and call
  // initialize instead
  ParticleCloudDisplay(
    rviz_common::DisplayContext * display_context,
    Ogre::SceneNode * scene_node);
  ParticleCloudDisplay();
  ~ParticleCloudDisplay() override;

  void processMessage(nav2_msgs::msg::ParticleCloud::ConstSharedPtr msg) override;
  void setShape(QString shape);  // for testing

protected:
  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  /// Update the interface and visible shapes based on the selected shape type.
  void updateShapeChoice();

  /// Update the arrow color.
  void updateArrowColor();

  /// Update arrow geometry
  void updateGeometry();

private:
  void initializeProperties();
  bool validateFloats(const nav2_msgs::msg::ParticleCloud & msg);
  bool setTransform(std_msgs::msg::Header const & header);
  void updateDisplay();
  void updateArrows2d();
  void updateArrows3d();
  void updateAxes();
  void updateArrow3dGeometry();
  void updateAxesGeometry();

  std::unique_ptr<rviz_rendering::Axes> makeAxes();
  std::unique_ptr<rviz_rendering::Arrow> makeArrow3d();

  std::vector<OgrePoseWithWeight> poses_;
  std::unique_ptr<FlatWeightedArrowsArray> arrows2d_;
  std::vector<std::unique_ptr<rviz_rendering::Arrow>> arrows3d_;
  std::vector<std::unique_ptr<rviz_rendering::Axes>> axes_;

  Ogre::SceneNode * arrow_node_;
  Ogre::SceneNode * axes_node_;

  rviz_common::properties::EnumProperty * shape_property_;
  rviz_common::properties::ColorProperty * arrow_color_property_;
  rviz_common::properties::FloatProperty * arrow_alpha_property_;

  rviz_common::properties::FloatProperty * arrow_min_length_property_;
  rviz_common::properties::FloatProperty * arrow_max_length_property_;

  float min_length_;
  float max_length_;
  float length_scale_;
  float head_radius_scale_;
  float head_length_scale_;
  float shaft_radius_scale_;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__PARTICLE_CLOUD_DISPLAY_HPP_
