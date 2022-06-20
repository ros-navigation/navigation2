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

#ifndef NAV2_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__FLAT_WEIGHTED_ARROWS_ARRAY_HPP_
#define NAV2_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__FLAT_WEIGHTED_ARROWS_ARRAY_HPP_

#include <vector>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include "nav2_rviz_plugins/particle_cloud_display/particle_cloud_display.hpp"

namespace nav2_rviz_plugins
{

struct OgrePoseWithWeight;

class FlatWeightedArrowsArray
{
public:
  explicit FlatWeightedArrowsArray(Ogre::SceneManager * scene_manager_);
  ~FlatWeightedArrowsArray();

  void createAndAttachManualObject(Ogre::SceneNode * scene_node);
  void updateManualObject(
    Ogre::ColourValue color,
    float alpha,
    float min_length,
    float max_length,
    const std::vector<nav2_rviz_plugins::OgrePoseWithWeight> & poses);
  void clear();

private:
  void setManualObjectMaterial();
  void setManualObjectVertices(
    const Ogre::ColourValue & color,
    float min_length,
    float max_length,
    const std::vector<nav2_rviz_plugins::OgrePoseWithWeight> & poses);

  Ogre::SceneManager * scene_manager_;
  Ogre::ManualObject * manual_object_;
  Ogre::MaterialPtr material_;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__FLAT_WEIGHTED_ARROWS_ARRAY_HPP_
