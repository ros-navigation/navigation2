// Copyright (c) 2025 Dexory
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

#ifndef NAV2_PLANNER__IS_PATH_VALID_SERVICE_HPP_
#define NAV2_PLANNER__IS_PATH_VALID_SERVICE_HPP_

#include <memory>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_ros_common/service_server.hpp"
#include "tf2/utils.hpp"

namespace nav2_planner
{

/**
 * @class nav2_planner::IsPathValidService
 * @brief Service to determine if a path is still valid given the current costmap state
 */
class IsPathValidService
{
public:
  /**
   * @brief Constructor for IsPathValidService
   * @param node Lifecycle node pointer
   * @param costmap_ros Costmap ROS wrapper
   * @param costmap_update_timeout Timeout for waiting for costmap updates
   */
  IsPathValidService(
    nav2::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    const rclcpp::Duration & costmap_update_timeout)
  : node_(node), costmap_ros_(costmap_ros), logger_(rclcpp::get_logger("is_path_valid_service")),
    costmap_update_timeout_(costmap_update_timeout)
  {
  }

  /**
   * @brief Initialize the service
   */
  void initialize()
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("Failed to lock node in initialize");
    }

    costmap_ = costmap_ros_->getCostmap();

    service_ = node->create_service<nav2_msgs::srv::IsPathValid>(
      "is_path_valid",
      std::bind(
        &IsPathValidService::callback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  /**
   * @brief Reset the service
   */
  void reset()
  {
    service_.reset();
    costmap_ = nullptr;
  }

private:
  /**
   * @brief Get the costmap to check based on the layer name
   * @param layer_name Name of the layer to check, or empty for full costmap
   * @return Pointer to the costmap to check, or nullptr if layer not found
   */
  nav2_costmap_2d::Costmap2D * getCostmapToCheck(const std::string & layer_name)
  {
    if (layer_name.empty()) {
      return costmap_;
    }

    auto layers = costmap_ros_->getLayeredCostmap()->getPlugins();
    for (auto & layer : *layers) {
      if (layer->getName() == layer_name) {
        auto * costmap_layer = dynamic_cast<nav2_costmap_2d::CostmapLayer *>(layer.get());
        if (costmap_layer != nullptr) {
          return static_cast<nav2_costmap_2d::Costmap2D *>(costmap_layer);
        }
      }
    }

    RCLCPP_ERROR(
      logger_, "Requested layer '%s' not found or does not provide a costmap.",
      layer_name.c_str());
    return nullptr;
  }

  /**
   * @brief Get the footprint to use for collision checking
   * @param footprint_string Custom footprint string, or empty to use robot's footprint
   * @param footprint Output parameter for the footprint
   * @param use_radius Output parameter indicating if radius-based checking should be used
   * @return True if successful, false if footprint string is invalid
   */
  bool getFootprintToUse(
    const std::string & footprint_string,
    nav2_costmap_2d::Footprint & footprint,
    bool & use_radius)
  {
    use_radius = costmap_ros_->getUseRadius();

    if (!footprint_string.empty()) {
      if (!nav2_costmap_2d::makeFootprintFromString(footprint_string, footprint)) {
        RCLCPP_ERROR(
          logger_, "Invalid footprint string '%s'. Cannot validate path.",
          footprint_string.c_str());
        return false;
      }
      use_radius = false;
    } else if (!use_radius) {
      footprint = costmap_ros_->getRobotFootprint();
    }

    return true;
  }

  /**
   * @brief Find the closest point on the path to the current pose
   * @param current_pose The current robot pose
   * @param path The path to search
   * @return Index of the closest pose in the path
   */
  unsigned int findClosestPointIndex(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & path)
  {
    unsigned int closest_point_index = 0;
    float closest_distance = std::numeric_limits<float>::max();
    const auto & current_point = current_pose.pose.position;

    for (unsigned int i = 0; i < path.poses.size(); ++i) {
      const auto & path_point = path.poses[i].pose.position;
      float distance = nav2_util::geometry_utils::euclidean_distance(current_point, path_point);

      if (distance < closest_distance) {
        closest_point_index = i;
        closest_distance = distance;
      }
    }

    return closest_point_index;
  }

  /**
   * @brief Service callback to determine if the path is still valid
   */
  void callback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
    std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
  {
    response->success = true;
    response->is_valid = true;

    if (request->path.poses.empty()) {
      RCLCPP_ERROR(logger_, "Received empty path. Cannot validate path.");
      response->success = false;
      response->is_valid = false;
      return;
    }

    // Wait for costmap to become current (skip if timeout is zero or negative)
    if (costmap_update_timeout_ > rclcpp::Duration(0, 0)) {
      try {
        costmap_ros_->waitUntilCurrent(costmap_update_timeout_);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(logger_, "Failed to wait for costmap: %s", ex.what());
        response->success = false;
        response->is_valid = false;
        return;
      }
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!costmap_ros_->getRobotPose(current_pose)) {
      RCLCPP_ERROR(logger_, "Failed to get robot pose. Cannot validate path.");
      response->success = false;
      response->is_valid = false;
      return;
    }

    /**
     * The lethal check starts at the closest point to avoid points that have already been passed
     * and may have become occupied. The method for collision detection is based on the shape of
     * the footprint.
     */
    unsigned int closest_point_index = findClosestPointIndex(current_pose, request->path);

    // Determine which costmap to use based on layer_name parameter
    nav2_costmap_2d::Costmap2D * costmap_to_check = getCostmapToCheck(request->layer_name);

    if (!costmap_to_check) {
      response->success = false;
      response->is_valid = false;
      return;
    }

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap_to_check->getMutex()));
    unsigned int mx = 0;
    unsigned int my = 0;

    // Determine footprint to use
    nav2_costmap_2d::Footprint footprint;
    bool use_radius;

    if (!getFootprintToUse(request->footprint, footprint, use_radius)) {
      response->success = false;
      response->is_valid = false;
      return;
    }

    // Create collision checker for this request if needed
    std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
    collision_checker;
    if (!use_radius) {
      collision_checker =
        std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
        costmap_to_check);
    }

    unsigned int cost = nav2_costmap_2d::FREE_SPACE;
    for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
      auto & position = request->path.poses[i].pose.position;
      if (use_radius) {
        if (costmap_to_check->worldToMap(position.x, position.y, mx, my)) {
          cost = costmap_to_check->getCost(mx, my);
        } else {
          cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        }
      } else {
        auto theta = tf2::getYaw(request->path.poses[i].pose.orientation);
        cost = static_cast<unsigned int>(collision_checker->footprintCostAtPose(
            position.x, position.y, theta, footprint));
      }

      if (cost == nav2_costmap_2d::NO_INFORMATION && request->consider_unknown_as_obstacle) {
        cost = nav2_costmap_2d::LETHAL_OBSTACLE;
      } else if (cost == nav2_costmap_2d::NO_INFORMATION) {
        cost = nav2_costmap_2d::FREE_SPACE;
      }

      if (use_radius &&
        (cost >= request->max_cost || cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
      {
        response->is_valid = false;
        response->invalid_pose_indices.push_back(i);
        if (!request->check_full_path) {
          break;
        }
      } else if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost >= request->max_cost) {
        response->is_valid = false;
        response->invalid_pose_indices.push_back(i);
        if (!request->check_full_path) {
          break;
        }
      }
    }
  }

  nav2::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_;
  rclcpp::Duration costmap_update_timeout_;
  nav2::ServiceServer<nav2_msgs::srv::IsPathValid>::SharedPtr service_;
};

}  // namespace nav2_planner

#endif  // NAV2_PLANNER__IS_PATH_VALID_SERVICE_HPP_
