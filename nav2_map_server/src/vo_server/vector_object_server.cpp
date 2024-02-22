// Copyright (c) 2023 Samsung R&D Institute Russia
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

#include "nav2_map_server/vector_object_server.hpp"

#include <uuid/uuid.h>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "tf2_ros/create_timer_ros.h"

#include "nav2_util/occ_grid_utils.hpp"

using namespace std::placeholders;

namespace nav2_map_server
{

VectorObjectServer::VectorObjectServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("vector_object_server", "", options), process_map_(false)
{}

nav2_util::CallbackReturn
VectorObjectServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Obtaining ROS parameters
  if (!getROSParameters()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Make name prefix for services
  const std::string service_prefix = get_name() + std::string("/");

  add_shapes_service_ = create_service<nav2_msgs::srv::AddShapes>(
    service_prefix + std::string("add_shapes"),
    std::bind(&VectorObjectServer::addShapesCallback, this, _1, _2, _3));

  get_shapes_service_ = create_service<nav2_msgs::srv::GetShapes>(
    service_prefix + std::string("get_shapes"),
    std::bind(&VectorObjectServer::getShapesCallback, this, _1, _2, _3));

  remove_shapes_service_ = create_service<nav2_msgs::srv::RemoveShapes>(
    service_prefix + std::string("remove_shapes"),
    std::bind(&VectorObjectServer::removeShapesCallback, this, _1, _2, _3));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VectorObjectServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  map_pub_->on_activate();

  // Creating bond connection
  createBond();

  // Trigger map to be published
  process_map_ = true;
  switchMapUpdate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VectorObjectServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (map_timer_) {
    map_timer_->cancel();
    map_timer_.reset();
  }
  process_map_ = false;

  map_pub_->on_deactivate();

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VectorObjectServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  add_shapes_service_.reset();
  get_shapes_service_.reset();
  remove_shapes_service_.reset();

  map_pub_.reset();
  map_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
VectorObjectServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}

bool VectorObjectServer::transformVectorObjects()
{
  for (auto shape : shapes_) {
    if (shape->getFrameID() != frame_id_ && !shape->getFrameID().empty()) {
      // Shape to be updated dynamically
      if (!shape->toFrame(frame_id_, tf_buffer_, transform_tolerance_)) {
        RCLCPP_ERROR(
          get_logger(), "Can not transform vector object from %s to %s frame",
          shape->getFrameID().c_str(), frame_id_.c_str());
        return false;
      }
    }
  }

  return true;
}

void VectorObjectServer::getMapBoundaries(
  double & min_x, double & min_y, double & max_x, double & max_y)
{
  min_x = std::numeric_limits<double>::max();
  min_y = std::numeric_limits<double>::max();
  max_x = std::numeric_limits<double>::lowest();
  max_y = std::numeric_limits<double>::lowest();

  double min_p_x, min_p_y, max_p_x, max_p_y;

  for (auto shape : shapes_) {
    shape->getBoundaries(min_p_x, min_p_y, max_p_x, max_p_y);
    if (min_p_x < min_x) {
      min_x = min_p_x;
    }
    if (min_p_y < min_y) {
      min_y = min_p_y;
    }
    if (max_p_x > max_x) {
      max_x = max_p_x;
    }
    if (max_p_y > max_y) {
      max_y = max_p_y;
    }
  }

  if (
    min_x == std::numeric_limits<double>::max() ||
    min_y == std::numeric_limits<double>::max() ||
    max_x == std::numeric_limits<double>::lowest() ||
    max_y == std::numeric_limits<double>::lowest())
  {
    throw std::runtime_error("Can not obtain map boundaries");
  }
}

void VectorObjectServer::updateMap(
  const double & min_x, const double & min_y, const double & max_x, const double & max_y)
{
  // Calculate size of update map
  int size_x = static_cast<int>((max_x - min_x) / resolution_) + 1;
  int size_y = static_cast<int>((max_y - min_y) / resolution_) + 1;

  if (size_x < 0) {
    throw std::runtime_error("Incorrect map x-size");
  }
  if (size_y < 0) {
    throw std::runtime_error("Incorrect map y-size");
  }

  if (!map_) {
    map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  }
  if (
    map_->info.width != static_cast<unsigned int>(size_x) ||
    map_->info.height != static_cast<unsigned int>(size_y))
  {
    // Map size was changed
    map_->data = std::vector<int8_t>(size_x * size_y, default_value_);
    map_->info.width = size_x;
    map_->info.height = size_y;
  } else if (size_x > 0 && size_y > 0) {
    // Map size was not changed
    memset(map_->data.data(), default_value_, size_x * size_y * sizeof(int8_t));
  }
  map_->header.frame_id = frame_id_;
  map_->info.resolution = resolution_;
  map_->info.origin.position.x = min_x;
  map_->info.origin.position.y = min_y;
}

void VectorObjectServer::putVectorObjectsOnMap()
{
  // Filling the shapes
  for (auto shape : shapes_) {
    if (shape->isFill()) {
      // Put filled shape on map
      double wx1 = std::numeric_limits<double>::max();
      double wy1 = std::numeric_limits<double>::max();
      double wx2 = std::numeric_limits<double>::lowest();
      double wy2 = std::numeric_limits<double>::lowest();
      unsigned int mx1 = 0;
      unsigned int my1 = 0;
      unsigned int mx2 = 0;
      unsigned int my2 = 0;

      shape->getBoundaries(wx1, wy1, wx2, wy2);
      if (
        !nav2_util::worldToMap(map_, wx1, wy1, mx1, my1) ||
        !nav2_util::worldToMap(map_, wx2, wy2, mx2, my2))
      {
        RCLCPP_ERROR(
          get_logger(),
          "Error to get shape boundaries on map (UUID: %s)",
          shape->getUUID().c_str());
        return;
      }

      unsigned int it;
      for (unsigned int my = my1; my <= my2; my++) {
        for (unsigned int mx = mx1; mx <= mx2; mx++) {
          it = my * map_->info.width + mx;
          double wx, wy;
          nav2_util::mapToWorld(map_, mx, my, wx, wy);
          if (shape->isPointInside(wx, wy)) {
            processVal(map_->data[it], shape->getValue(), overlay_type_);
          }
        }
      }
    } else {
      // Put shape borders on map
      shape->putBorders(map_, overlay_type_);
    }
  }
}

void VectorObjectServer::publishMap()
{
  if (map_) {
    auto map = std::make_unique<nav_msgs::msg::OccupancyGrid>(*map_);
    map_pub_->publish(std::move(map));
  }
}

void VectorObjectServer::processMap()
{
  if (!process_map_) {
    return;
  }

  try {
    if (shapes_.size()) {
      if (!transformVectorObjects()) {
        return;
      }
      double min_x, min_y, max_x, max_y;

      getMapBoundaries(min_x, min_y, max_x, max_y);
      updateMap(min_x, min_y, max_x, max_y);
      putVectorObjectsOnMap();
    } else {
      updateMap(0.0, 0.0, 0.0, 0.0);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Can not upate map: %s", ex.what());
    return;
  }

  publishMap();
}

bool VectorObjectServer::isMapUpdate()
{
  for (auto shape : shapes_) {
    if (shape->getFrameID() != frame_id_ && !shape->getFrameID().empty()) {
      return true;
    }
  }

  return false;
}

void VectorObjectServer::switchMapUpdate()
{
  if (isMapUpdate()) {
    if (!map_timer_) {
      map_timer_ = this->create_timer(
        std::chrono::duration<double>(1.0 / update_frequency_),
        std::bind(&VectorObjectServer::processMap, this));
      RCLCPP_INFO(get_logger(), "Publishing map dynamically");
    }
  } else {
    if (map_timer_) {
      map_timer_->cancel();
      map_timer_.reset();
    }
    RCLCPP_INFO(get_logger(), "Publishing map once");
    processMap();
  }
}

void VectorObjectServer::addShapesCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::AddShapes::Request> request,
  std::shared_ptr<nav2_msgs::srv::AddShapes::Response> response)
{
  // Initialize result with true. If one of the required vector object was not added properly,
  // set it to false.
  response->success = true;

  auto node = shared_from_this();

  // Process polygons
  for (auto req_poly : request->polygons) {
    nav2_msgs::msg::PolygonVO::SharedPtr new_params =
      std::make_shared<nav2_msgs::msg::PolygonVO>(req_poly);

    bool new_shape = true;  // Whether to add a new shape
    std::shared_ptr<Polygon> polygon;
    for (auto shape : shapes_) {
      if (shape->isUUID(new_params->uuid.uuid.data())) {
        // Vector Object with given UUID was found: updating it
        new_shape = false;

        // Check that found shape has correct type
        if (shape->getType() != POLYGON) {
          RCLCPP_ERROR(
            get_logger(),
            "Shape (UUID: %s) is not a polygon type",
            shape->getUUID().c_str());
          response->success = false;
          // Do not add this shape
          break;
        }

        polygon = std::static_pointer_cast<Polygon>(shape);

        // Preserving old parameters for the case, if new ones to be incorrect
        nav2_msgs::msg::PolygonVO::SharedPtr old_params = polygon->getParams();
        try {
          polygon->setParams(new_params);
        } catch (const std::exception & ex) {
          // Restore old parameters
          polygon->setParams(old_params);
          // ... and report the problem
          RCLCPP_ERROR(get_logger(), "Can not update polygon: %s", ex.what());
          response->success = false;
        }
        break;
      }
    }

    if (new_shape) {
      // Creating new polygon
      try {
        polygon = std::make_shared<Polygon>(node, new_params);
        shapes_.push_back(polygon);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Can not add polygon: %s", ex.what());
        response->success = false;
      }
    }
  }

  // Process circles
  std::shared_ptr<Circle> circle;
  for (auto req_crcl : request->circles) {
    nav2_msgs::msg::CircleVO::SharedPtr new_params =
      std::make_shared<nav2_msgs::msg::CircleVO>(req_crcl);

    bool new_shape = true;  // Whether to add a new shape
    for (auto shape : shapes_) {
      if (shape->isUUID(new_params->uuid.uuid.data())) {
        // Vector object with given UUID was found: updating it
        new_shape = false;

        // Check that found shape has correct type
        if (shape->getType() != CIRCLE) {
          RCLCPP_ERROR(
            get_logger(),
            "Shape (UUID: %s) is not a circle type",
            shape->getUUID().c_str());
          response->success = false;
          break;
        }

        circle = std::static_pointer_cast<Circle>(shape);

        // Preserving old parameters for the case, if new ones to be incorrect
        nav2_msgs::msg::CircleVO::SharedPtr old_params = circle->getParams();
        try {
          circle->setParams(new_params);
        } catch (const std::exception & ex) {
          // Restore old parameters
          circle->setParams(old_params);
          // ... and report the problem
          RCLCPP_ERROR(get_logger(), "Can not update circle: %s", ex.what());
          response->success = false;
        }
        break;
      }
    }

    if (new_shape) {
      // Creating new circle
      try {
        circle = std::make_shared<Circle>(node, new_params);
        shapes_.push_back(circle);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Can not add circle: %s", ex.what());
        response->success = false;
      }
    }
  }

  switchMapUpdate();
}

void VectorObjectServer::getShapesCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::GetShapes::Request>/*request*/,
  std::shared_ptr<nav2_msgs::srv::GetShapes::Response> response)
{
  std::shared_ptr<Polygon> polygon;
  std::shared_ptr<Circle> circle;

  for (auto shape : shapes_) {
    switch (shape->getType()) {
      case POLYGON:
        polygon = std::static_pointer_cast<Polygon>(shape);
        response->polygons.push_back(*(polygon->getParams()));
        break;
      case CIRCLE:
        circle = std::static_pointer_cast<Circle>(shape);
        response->circles.push_back(*(circle->getParams()));
        break;
      default:
        RCLCPP_WARN(get_logger(), "Unknown shape type (UUID: %s)", shape->getUUID().c_str());
    }
  }
}

void VectorObjectServer::removeShapesCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::RemoveShapes::Request> request,
  std::shared_ptr<nav2_msgs::srv::RemoveShapes::Response> response)
{
  // Initialize result with true. If one of the required vector object was not found,
  // set it to false.
  response->success = true;

  for (auto req_uuid : request->uuids) {
    bool found = false;

    for (
      std::vector<std::shared_ptr<Shape>>::iterator it = shapes_.begin();
      it != shapes_.end();
      it++)
    {
      if ((*it)->isUUID(req_uuid.uuid.data())) {
        // Polygon with given UUID was found: remove it
        (*it).reset();
        shapes_.erase(it);
        found = true;

        break;
      }
    }

    if (!found) {
      // Required vector object was not found
      char uuid_str[37];
      uuid_unparse(req_uuid.uuid.data(), uuid_str);
      RCLCPP_ERROR(get_logger(), "Can not find shape to remove with UUID: %s", uuid_str);
      response->success = false;
    }
  }

  switchMapUpdate();
}

bool VectorObjectServer::getROSParameters()
{
  // Main ROS-parameters
  map_topic_ = getROSParameter(
    shared_from_this(), "map_topic", "vo_map").as_string();
  frame_id_ = getROSParameter(
    shared_from_this(), "frame_id", "map").as_string();
  resolution_ = getROSParameter(
    shared_from_this(), "resolution", 0.05).as_double();
  default_value_ = getROSParameter(
    shared_from_this(), "default_value", nav2_util::OCC_GRID_UNKNOWN).as_int();
  overlay_type_ = static_cast<OverlayType>(getROSParameter(
      shared_from_this(), "overlay_type", static_cast<int>(OverlayType::OVERLAY_SEQ)).as_int());
  update_frequency_ = getROSParameter(
    shared_from_this(), "update_frequency", 1.0).as_double();
  transform_tolerance_ = getROSParameter(
    shared_from_this(), "transform_tolerance", 0.1).as_double();

  // Shapes
  std::vector<std::string> shape_names = getROSParameter(
    shared_from_this(), "shapes", std::vector<std::string>()).as_string_array();
  for (std::string shape_name : shape_names) {
    std::string shape_type;

    try {
      shape_type = getROSParameter(
        shared_from_this(), shape_name + ".type", rclcpp::PARAMETER_STRING).as_string();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(),
        "Error while getting shape %s type: %s",
        shape_name.c_str(), ex.what());
      return false;
    }

    if (shape_type == "polygon") {
      try {
        std::shared_ptr<Polygon> polygon = std::make_shared<Polygon>(shared_from_this());
        if (!polygon->getROSParameters(shape_name)) {
          return false;
        }
        shapes_.push_back(polygon);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Can not create new polygon: %s", ex.what());
        return false;
      }
    } else if (shape_type == "circle") {
      try {
        std::shared_ptr<Circle> circle = std::make_shared<Circle>(shared_from_this());

        if (!circle->getROSParameters(shape_name)) {
          return false;
        }
        shapes_.push_back(circle);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Can not create new circle: %s", ex.what());
        return false;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Please specify correct shape %s type", shape_name.c_str());
    }
  }

  return true;
}

}  // namespace nav2_map_server
