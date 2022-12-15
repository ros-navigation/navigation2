/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "nav2_rviz_plugins/path_with_cost_display.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_common/validate_floats.hpp"

#include "rviz_rendering/material_manager.hpp"

namespace rviz_default_plugins
{
namespace displays
{

PathWithCostDisplay::PathWithCostDisplay(rviz_common::DisplayContext * context)
    : PathWithCostDisplay()
{
  context_ = context;
  scene_manager_ = context->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  updateBufferLength();
}

PathWithCostDisplay::PathWithCostDisplay()
{
  style_property_ = new rviz_common::properties::EnumProperty(
      "Line Style", "Lines",
      "The rendering operation to use to draw the grid lines.",
      this, SLOT(updateStyle()));

  style_property_->addOption("Lines", LINES);
  style_property_->addOption("Billboards", BILLBOARDS);

  line_width_property_ = new rviz_common::properties::FloatProperty(
      "Line Width", 0.03f,
      "The width, in meters, of each path line."
      "Only works with the 'Billboards' style.",
      this, SLOT(updateLineWidth()), this);
  line_width_property_->setMin(0.001f);
  line_width_property_->hide();

  color_property_ = new rviz_common::properties::ColorProperty(
      "Color", QColor(25, 255, 0),
      "Color to draw the path.", this);

  alpha_property_ = new rviz_common::properties::FloatProperty(
      "Alpha", 1.0,
      "Amount of transparency to apply to the path.", this);

  buffer_length_property_ = new rviz_common::properties::IntProperty(
      "Buffer Length", 1,
      "Number of paths to display.",
      this, SLOT(updateBufferLength()));
  buffer_length_property_->setMin(1);

  offset_property_ = new rviz_common::properties::VectorProperty(
      "Offset", Ogre::Vector3::ZERO,
      "Allows you to offset the path from the origin of the reference frame.  In meters.",
      this, SLOT(updateOffset()));

  pose_style_property_ = new rviz_common::properties::EnumProperty(
      "Pose Style", "None",
      "Shape to display the pose as.",
      this, SLOT(updatePoseStyle()));
  pose_style_property_->addOption("None", NONE);
  pose_style_property_->addOption("Axes", AXES);
  pose_style_property_->addOption("Arrows", ARROWS);

  pose_axes_length_property_ = new rviz_common::properties::FloatProperty(
      "Length", 0.3f,
      "Length of the axes.",
      this, SLOT(updatePoseAxisGeometry()) );
  pose_axes_radius_property_ = new rviz_common::properties::FloatProperty(
      "Radius", 0.03f,
      "Radius of the axes.",
      this, SLOT(updatePoseAxisGeometry()) );

  pose_arrow_color_property_ = new rviz_common::properties::ColorProperty(
      "Pose Color",
      QColor(255, 85, 255),
      "Color to draw the poses.",
      this, SLOT(updatePoseArrowColor()));
  pose_arrow_shaft_length_property_ = new rviz_common::properties::FloatProperty(
      "Shaft Length",
      0.1f,
      "Length of the arrow shaft.",
      this,
      SLOT(updatePoseArrowGeometry()));
  pose_arrow_head_length_property_ = new rviz_common::properties::FloatProperty(
      "Head Length", 0.2f,
      "Length of the arrow head.",
      this,
      SLOT(updatePoseArrowGeometry()));
  pose_arrow_shaft_diameter_property_ = new rviz_common::properties::FloatProperty(
      "Shaft Diameter",
      0.1f,
      "Diameter of the arrow shaft.",
      this,
      SLOT(updatePoseArrowGeometry()));
  pose_arrow_head_diameter_property_ = new rviz_common::properties::FloatProperty(
      "Head Diameter",
      0.3f,
      "Diameter of the arrow head.",
      this,
      SLOT(updatePoseArrowGeometry()));
  pose_axes_length_property_->hide();
  pose_axes_radius_property_->hide();
  pose_arrow_color_property_->hide();
  pose_arrow_shaft_length_property_->hide();
  pose_arrow_head_length_property_->hide();
  pose_arrow_shaft_diameter_property_->hide();
  pose_arrow_head_diameter_property_->hide();

  static int count = 0;
  std::string material_name = "LinesMaterial" + std::to_string(count++);
  lines_material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);
}

PathWithCostDisplay::~PathWithCostDisplay()
{
  destroyObjects();
  destroyPoseAxesChain();
  destroyPoseArrowChain();
}

void PathWithCostDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateBufferLength();
}

void PathWithCostDisplay::reset()
{
  MFDClass::reset();
  updateBufferLength();
}


void PathWithCostDisplay::allocateAxesVector(std::vector<rviz_rendering::Axes *> & axes_vect, size_t num)
{
  auto vector_size = axes_vect.size();
  if (num > vector_size) {
    axes_vect.reserve(num);
    for (auto i = vector_size; i < num; ++i) {
      axes_vect.push_back(
          new rviz_rendering::Axes(
              scene_manager_, scene_node_,
              pose_axes_length_property_->getFloat(),
              pose_axes_radius_property_->getFloat()));
    }
  } else if (num < vector_size) {
    for (auto i = num; i < vector_size; ++i) {
      delete axes_vect[i];
    }
    axes_vect.resize(num);
  }
}

void PathWithCostDisplay::allocateArrowVector(std::vector<rviz_rendering::Arrow *> & arrow_vect, size_t num)
{
  auto vector_size = arrow_vect.size();
  if (num > vector_size) {
    arrow_vect.reserve(num);
    for (auto i = vector_size; i < num; ++i) {
      arrow_vect.push_back(new rviz_rendering::Arrow(scene_manager_, scene_node_));
    }
  } else if (num < vector_size) {
    for (auto i = num; i < vector_size; ++i) {
      delete arrow_vect[i];
    }
    arrow_vect.resize(num);
  }
}

void PathWithCostDisplay::destroyPoseAxesChain()
{
  for (auto & axes_vect : axes_chain_) {
    allocateAxesVector(axes_vect, 0);
  }
  axes_chain_.clear();
}

void PathWithCostDisplay::destroyPoseArrowChain()
{
  for (auto & arrow_vect : arrow_chain_) {
    allocateArrowVector(arrow_vect, 0);
  }
  arrow_chain_.clear();
}

void PathWithCostDisplay::updateStyle()
{
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());

  if (style == BILLBOARDS) {
    line_width_property_->show();
  } else {
    line_width_property_->hide();
  }

  updateBufferLength();
}

void PathWithCostDisplay::updateLineWidth()
{
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());
  float line_width = line_width_property_->getFloat();

  if (style == BILLBOARDS) {
    for (auto billboard_line : billboard_lines_) {
      if (billboard_line) {
        billboard_line->setLineWidth(line_width);
      }
    }
  }
  context_->queueRender();
}

void PathWithCostDisplay::updateOffset()
{
  scene_node_->setPosition(offset_property_->getVector() );
  context_->queueRender();
}

void PathWithCostDisplay::updatePoseStyle()
{
  auto pose_style = static_cast<PoseStyle>(pose_style_property_->getOptionInt());
  switch (pose_style) {
    case AXES:
      pose_axes_length_property_->show();
      pose_axes_radius_property_->show();
      pose_arrow_color_property_->hide();
      pose_arrow_shaft_length_property_->hide();
      pose_arrow_head_length_property_->hide();
      pose_arrow_shaft_diameter_property_->hide();
      pose_arrow_head_diameter_property_->hide();
      break;
    case ARROWS:
      pose_axes_length_property_->hide();
      pose_axes_radius_property_->hide();
      pose_arrow_color_property_->show();
      pose_arrow_shaft_length_property_->show();
      pose_arrow_head_length_property_->show();
      pose_arrow_shaft_diameter_property_->show();
      pose_arrow_head_diameter_property_->show();
      break;
    default:
      pose_axes_length_property_->hide();
      pose_axes_radius_property_->hide();
      pose_arrow_color_property_->hide();
      pose_arrow_shaft_length_property_->hide();
      pose_arrow_head_length_property_->hide();
      pose_arrow_shaft_diameter_property_->hide();
      pose_arrow_head_diameter_property_->hide();
  }
  updateBufferLength();
}

void PathWithCostDisplay::updatePoseAxisGeometry()
{
  for (auto & axes_vect : axes_chain_) {
    for (auto & axes : axes_vect) {
      axes->set(
          pose_axes_length_property_->getFloat(),
          pose_axes_radius_property_->getFloat() );
    }
  }
  context_->queueRender();
}

void PathWithCostDisplay::updatePoseArrowColor()
{
  QColor color = pose_arrow_color_property_->getColor();

  for (auto & arrow_vect : arrow_chain_) {
    for (auto & arrow : arrow_vect) {
      arrow->setColor(color.redF(), color.greenF(), color.blueF(), 1.0f);
    }
  }
  context_->queueRender();
}

void PathWithCostDisplay::updatePoseArrowGeometry()
{
  for (auto & arrow_vect : arrow_chain_) {
    for (auto & arrow : arrow_vect) {
      arrow->set(
          pose_arrow_shaft_length_property_->getFloat(),
          pose_arrow_shaft_diameter_property_->getFloat(),
          pose_arrow_head_length_property_->getFloat(),
          pose_arrow_head_diameter_property_->getFloat());
    }
  }
  context_->queueRender();
}

void PathWithCostDisplay::destroyObjects()
{
  // Destroy all simple lines, if any
  for (auto manual_object : manual_objects_) {
    manual_object->clear();
    scene_manager_->destroyManualObject(manual_object);
  }
  manual_objects_.clear();

  // Destroy all billboards, if any
  for (auto billboard_line : billboard_lines_) {
    delete billboard_line;  // also destroys the corresponding scene node
  }
  billboard_lines_.clear();
}

void PathWithCostDisplay::updateBufferLength()
{
  // Destroy all path objects
  destroyObjects();

  // Destroy all axes and arrows
  destroyPoseAxesChain();
  destroyPoseArrowChain();

  // Read options
  auto buffer_length = static_cast<size_t>(buffer_length_property_->getInt());
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());

  // Create new path objects
  switch (style) {
    case LINES:  // simple lines with fixed width of 1px
      manual_objects_.reserve(buffer_length);
      for (size_t i = 0; i < buffer_length; i++) {
        auto manual_object = scene_manager_->createManualObject();
        manual_object->setDynamic(true);
        scene_node_->attachObject(manual_object);

        manual_objects_.push_back(manual_object);
      }
      break;

    case BILLBOARDS:  // billboards with configurable width
      billboard_lines_.reserve(buffer_length);
      for (size_t i = 0; i < buffer_length; i++) {
        auto billboard_line = new rviz_rendering::BillboardLine(scene_manager_, scene_node_);
        billboard_lines_.push_back(billboard_line);
      }
      break;
  }
  axes_chain_.resize(buffer_length);
  arrow_chain_.resize(buffer_length);
}

bool validateFloats(const nav2_msgs::msg::PathWithCost & msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.poses);
  return valid;
}

void PathWithCostDisplay::processMessage(nav2_msgs::msg::PathWithCost::ConstSharedPtr msg)
{
  // Calculate index of oldest element in cyclic buffer
  size_t bufferIndex = messages_received_ % buffer_length_property_->getInt();

  auto style = static_cast<LineStyle>(style_property_->getOptionInt());
  Ogre::ManualObject * manual_object = nullptr;
  rviz_rendering::BillboardLine * billboard_line = nullptr;

  // Delete oldest element
  switch (style) {
    case LINES:
      manual_object = manual_objects_[bufferIndex];
      manual_object->clear();
      break;

    case BILLBOARDS:
      billboard_line = billboard_lines_[bufferIndex];
      billboard_line->clear();
      break;
  }

  // Check if path contains invalid coordinate values
  if (!validateFloats(*msg)) {
    setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic", "Message contained invalid "
                                                                 "floating point "
                                                                 "values (nans or infs)");
    return;
  }

  // Lookup transform into fixed frame
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  Ogre::Matrix4 transform(orientation);
  transform.setTrans(position);

  switch (style) {
    case LINES:
      updateManualObject(manual_object, msg, transform);
      break;

    case BILLBOARDS:
      updateBillBoardLine(billboard_line, msg, transform);
      break;
  }
  updatePoseMarkers(bufferIndex, msg, transform);

  context_->queueRender();
}

void PathWithCostDisplay::updateManualObject(
    Ogre::ManualObject * manual_object, nav2_msgs::msg::PathWithCost::ConstSharedPtr msg,
    const Ogre::Matrix4 & transform)
{
  auto color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  manual_object->estimateVertexCount(msg->poses.size());
  manual_object->begin(
      lines_material_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");

  for (auto pose_stamped : msg->poses) {
    manual_object->position(transform * rviz_common::pointMsgToOgre(pose_stamped.pose.position));
    rviz_rendering::MaterialManager::enableAlphaBlending(lines_material_, color.a);
    manual_object->colour(color);
  }

  manual_object->end();
}

void PathWithCostDisplay::updateBillBoardLine(
    rviz_rendering::BillboardLine * billboard_line, nav2_msgs::msg::PathWithCost::ConstSharedPtr msg,
    const Ogre::Matrix4 & transform)
{
  auto color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  billboard_line->setNumLines(1);
  billboard_line->setMaxPointsPerLine(static_cast<uint32_t>(msg->poses.size()));
  billboard_line->setLineWidth(line_width_property_->getFloat());

  for (auto pose_stamped : msg->poses) {
    Ogre::Vector3 xpos = transform * rviz_common::pointMsgToOgre(pose_stamped.pose.position);
    billboard_line->addPoint(xpos, color);
  }
}

void PathWithCostDisplay::updatePoseMarkers(
    size_t buffer_index, nav2_msgs::msg::PathWithCost::ConstSharedPtr msg, const Ogre::Matrix4 & transform)
{
  auto pose_style = static_cast<PoseStyle>(pose_style_property_->getOptionInt());
  auto & arrow_vect = arrow_chain_[buffer_index];
  auto & axes_vect = axes_chain_[buffer_index];

  if (pose_style == AXES) {
    updateAxesMarkers(axes_vect, msg, transform);
  }
  if (pose_style == ARROWS) {
    updateArrowMarkers(arrow_vect, msg, transform);
  }
}

void PathWithCostDisplay::updateAxesMarkers(
    std::vector<rviz_rendering::Axes *> & axes_vect, nav2_msgs::msg::PathWithCost::ConstSharedPtr msg,
    const Ogre::Matrix4 & transform)
{
  auto num_points = msg->poses.size();
  allocateAxesVector(axes_vect, num_points);
  for (size_t i = 0; i < num_points; ++i) {
    const geometry_msgs::msg::Point & pos = msg->poses[i].pose.position;
    axes_vect[i]->setPosition(transform * rviz_common::pointMsgToOgre(pos));
    Ogre::Quaternion orientation(rviz_common::quaternionMsgToOgre(msg->poses[i].pose.orientation));

    // Extract the rotation part of the transformation matrix as a quaternion
    Ogre::Quaternion transform_orientation = transform.linear();

    axes_vect[i]->setOrientation(transform_orientation * orientation);
  }
}

void PathWithCostDisplay::updateArrowMarkers(
    std::vector<rviz_rendering::Arrow *> & arrow_vect, nav2_msgs::msg::PathWithCost::ConstSharedPtr msg,
    const Ogre::Matrix4 & transform)
{
  auto num_points = msg->poses.size();
  allocateArrowVector(arrow_vect, num_points);
  for (size_t i = 0; i < num_points; ++i) {
    QColor color = pose_arrow_color_property_->getColor();
    arrow_vect[i]->setColor(color.redF(), color.greenF(), color.blueF(), 1.0f);

    arrow_vect[i]->set(
        pose_arrow_shaft_length_property_->getFloat(),
        pose_arrow_shaft_diameter_property_->getFloat(),
        pose_arrow_head_length_property_->getFloat(),
        pose_arrow_head_diameter_property_->getFloat());
    const geometry_msgs::msg::Point & pos = msg->poses[i].pose.position;
    arrow_vect[i]->setPosition(transform * rviz_common::pointMsgToOgre(pos));
    Ogre::Quaternion orientation(rviz_common::quaternionMsgToOgre(msg->poses[i].pose.orientation));

    // Extract the rotation part of the transformation matrix as a quaternion
    Ogre::Quaternion transform_orientation = transform.linear();

    Ogre::Vector3 dir(1, 0, 0);
    dir = transform_orientation * orientation * dir;
    arrow_vect[i]->setDirection(dir);
  }
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PathWithCostDisplay, rviz_common::Display)
