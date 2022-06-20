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

#include "nav2_rviz_plugins/particle_cloud_display/flat_weighted_arrows_array.hpp"

#include <vector>
#include <string>
#include <algorithm>

#include <OgreSceneManager.h>
#include <OgreTechnique.h>

#include "rviz_rendering/material_manager.hpp"

namespace nav2_rviz_plugins
{

FlatWeightedArrowsArray::FlatWeightedArrowsArray(Ogre::SceneManager * scene_manager)
: scene_manager_(scene_manager), manual_object_(nullptr) {}

FlatWeightedArrowsArray::~FlatWeightedArrowsArray()
{
  if (manual_object_) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void FlatWeightedArrowsArray::createAndAttachManualObject(Ogre::SceneNode * scene_node)
{
  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node->attachObject(manual_object_);
}

void FlatWeightedArrowsArray::updateManualObject(
  Ogre::ColourValue color,
  float alpha,
  float min_length,
  float max_length,
  const std::vector<nav2_rviz_plugins::OgrePoseWithWeight> & poses)
{
  clear();

  color.a = alpha;
  setManualObjectMaterial();
  rviz_rendering::MaterialManager::enableAlphaBlending(material_, alpha);

  manual_object_->begin(
    material_->getName(), Ogre::RenderOperation::OT_LINE_LIST, "rviz_rendering");
  setManualObjectVertices(color, min_length, max_length, poses);
  manual_object_->end();
}

void FlatWeightedArrowsArray::clear()
{
  if (manual_object_) {
    manual_object_->clear();
  }
}

void FlatWeightedArrowsArray::setManualObjectMaterial()
{
  static int material_count = 0;
  std::string material_name = "FlatWeightedArrowsMaterial" + std::to_string(material_count++);
  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);
}

void FlatWeightedArrowsArray::setManualObjectVertices(
  const Ogre::ColourValue & color,
  float min_length,
  float max_length,
  const std::vector<nav2_rviz_plugins::OgrePoseWithWeight> & poses)
{
  manual_object_->estimateVertexCount(poses.size() * 6);

  float scale = max_length - min_length;
  float length;
  for (const auto & pose : poses) {
    length = std::min(std::max(pose.weight * scale + min_length, min_length), max_length);
    Ogre::Vector3 vertices[6];
    vertices[0] = pose.position;  // back of arrow
    vertices[1] =
      pose.position + pose.orientation * Ogre::Vector3(length, 0, 0);  // tip of arrow
    vertices[2] = vertices[1];
    vertices[3] = pose.position + pose.orientation * Ogre::Vector3(
      0.75f * length, 0.2f * length, 0);
    vertices[4] = vertices[1];
    vertices[5] = pose.position + pose.orientation * Ogre::Vector3(
      0.75f * length, -0.2f * length,
      0);

    for (const auto & vertex : vertices) {
      manual_object_->position(vertex);
      manual_object_->colour(color);
    }
  }
}

}  // namespace nav2_rviz_plugins
