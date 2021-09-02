// Copyright (c) 2019 RoboTech Vision
// Copyright (c) 2019 Intel Corporation
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

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>

#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_smoother/nav2_smoother.hpp"
#include "tf2_ros/create_timer_ros.h"

using namespace std::chrono_literals;

namespace nav2_smoother
{

SmootherServer::SmootherServer()
: LifecycleNode("smoother_server", "", true),
  lp_loader_("nav2_core", "nav2_core::Smoother"),
  default_ids_{"SmoothPath"},
  default_types_{"nav2_smoother::CeresCostawareSmoother"}
{
  RCLCPP_INFO(get_logger(), "Creating smoother server");

  declare_parameter("smoother_plugins", default_ids_);
  declare_parameter("optimization_length", rclcpp::ParameterValue(8.0));
  declare_parameter("optimization_length_backwards", rclcpp::ParameterValue(4.0));
  declare_parameter("angular_distance_weight", rclcpp::ParameterValue(0.2));
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.5));
  declare_parameter("robot_frame_id", rclcpp::ParameterValue("base_footprint"));

  // // Launch a thread to run the costmap node
  // costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  RCLCPP_INFO(get_logger(), "Smoother server created");
}

SmootherServer::~SmootherServer()
{
  smoothers_.clear();
  // costmap_thread_.reset();
}

nav2_util::CallbackReturn
SmootherServer::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  get_parameter("smoother_plugins", smoother_ids_);
  get_parameter("optimization_length", optimization_length_);
  get_parameter("optimization_length_backwards", optimization_length_backwards_);
  get_parameter("transform_tolerance", transform_tolerance_);
  get_parameter("robot_frame_id", robot_frame_id_);
  get_parameter("angular_distance_weight", angular_distance_weight_);
  if (smoother_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  // Create the transform-related objects
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    rclcpp_node_->get_node_base_interface(),
    rclcpp_node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // The costmap node is used in the implementation of the controller
  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    shared_from_this(), "global_costmap/costmap_raw");
  
  smoother_types_.resize(smoother_ids_.size());

  for (size_t i = 0; i != smoother_ids_.size(); i++) {
    try {
      smoother_types_[i] = nav2_util::get_plugin_type_param(node, smoother_ids_[i]);
      nav2_core::Smoother::Ptr smoother =
        lp_loader_.createUniqueInstance(smoother_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created smoother : %s of type %s",
        smoother_ids_[i].c_str(), smoother_types_[i].c_str());
      smoother->configure(
        node, smoother_ids_[i],
        tf_buffer_, costmap_sub_);
      smoothers_.insert({smoother_ids_[i], smoother});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create smoother. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != smoother_ids_.size(); i++) {
    smoother_ids_concat_ += smoother_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Smoother Server has %s smoothers available.", smoother_ids_concat_.c_str());

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan_smoothed", 1);

  // Create the action server that we implement with our smoothPath method
  action_server_ = std::make_unique<ActionServer>(
    rclcpp_node_, "smooth_path",
    std::bind(&SmootherServer::smoothPlan, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SmootherServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  RCLCPP_INFO(get_logger(), "Checking transform");
  rclcpp::Rate r(2);

  plan_publisher_->on_activate();  
  SmootherMap::iterator it;
  for (it = smoothers_.begin(); it != smoothers_.end(); ++it) {
    it->second->activate();
  }
  action_server_->activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SmootherServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  SmootherMap::iterator it;
  for (it = smoothers_.begin(); it != smoothers_.end(); ++it) {
    it->second->deactivate();
  }
  plan_publisher_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SmootherServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  SmootherMap::iterator it;
  for (it = smoothers_.begin(); it != smoothers_.end(); ++it) {
    it->second->cleanup();
  }
  smoothers_.clear();

  // Release any allocated resources
  action_server_.reset();
  plan_publisher_.reset();
  tf_buffer_.reset();
  tf_listener_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SmootherServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool SmootherServer::findSmootherId(
  const std::string & c_name,
  std::string & current_smoother)
{
  if (smoothers_.find(c_name) == smoothers_.end()) {
    if (smoothers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No smoother was specified in action call."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", smoother_ids_concat_.c_str());
      current_smoother = smoothers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "SmoothPath called with smoother name %s, "
        "which does not exist. Available smoothers are: %s.",
        c_name.c_str(), smoother_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected smoother: %s.", c_name.c_str());
    current_smoother = c_name;
  }

  return true;
}

void SmootherServer::smoothPlan()
{
  RCLCPP_INFO(get_logger(), "Received a goal, smoothing.");

  auto result = std::make_shared<Action::Result>();
  try {
    std::string c_name = action_server_->get_current_goal()->smoother_id;
    std::string current_smoother;
    if (findSmootherId(c_name, current_smoother)) {
      current_smoother_ = current_smoother;
    } else {
      action_server_->terminate_current();
      return;
    }

    auto goal = action_server_->get_current_goal();
    std::size_t local_section_begin, local_section_end;
    findLocalSection(goal, local_section_begin, local_section_end);

    auto start_time = steady_clock_.now();

    // optimize path section
    result->path = goal->path;
    result->path.poses.erase(result->path.poses.begin() + local_section_begin, result->path.poses.begin() + local_section_end);
    nav_msgs::msg::Path local_section;
    local_section.header = goal->path.header;
    local_section.poses = std::vector<geometry_msgs::msg::PoseStamped>(goal->path.poses.begin() + local_section_begin,
                                                                       goal->path.poses.begin() + local_section_end);
    auto optimized_local_section = smoothers_[current_smoother_]->smoothPath(local_section).poses;
    result->path.poses.insert(result->path.poses.begin() + local_section_begin, optimized_local_section.begin(), optimized_local_section.end());
    result->path.header.stamp = now();

    auto cycle_duration = steady_clock_.now() - start_time;
    result->optimization_time = cycle_duration;

    RCLCPP_DEBUG(get_logger(), "Smoother succeeded (time: %lf), setting result", rclcpp::Duration(result->optimization_time).seconds());
    plan_publisher_->publish(result->path);

    action_server_->succeeded_current(result);
  } catch (nav2_core::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    action_server_->terminate_current();
    return;
  }
}

double SmootherServer::poseDistance(const geometry_msgs::msg::PoseStamped &pose1, const geometry_msgs::msg::PoseStamped &pose2) {
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  tf2::Quaternion q1;
  tf2::convert(pose1.pose.orientation, q1);
  tf2::Quaternion q2;
  tf2::convert(pose2.pose.orientation, q2);
  double da = angular_distance_weight_*std::abs(q1.angleShortestPath(q2));
  return std::sqrt(dx * dx + dy * dy + da * da);
}

void SmootherServer::findLocalSection(const std::shared_ptr<const typename Action::Goal> &goal, std::size_t &begin, std::size_t &end)
{
  RCLCPP_DEBUG(
    get_logger(),
    "Providing path to the smoother %s", current_smoother_.c_str());
  
  const nav_msgs::msg::Path &path = goal->path;

  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Invalid path, Path is empty.");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (goal->use_start) {
    robot_pose = goal->start;
  }
  else {
    try {
      rclcpp::Duration tolerance(rclcpp::Duration::from_seconds(transform_tolerance_));
      geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(costmap_sub_->getHeader().frame_id, robot_frame_id_, now(), tolerance);
      robot_pose.header = transform.header;
      robot_pose.pose.position.x = transform.transform.translation.x;
      robot_pose.pose.position.y = transform.transform.translation.y;
      robot_pose.pose.position.z = transform.transform.translation.z;
      robot_pose.pose.orientation = transform.transform.rotation;
    } catch (tf2::TransformException &ex) {
      std::stringstream ss;
      ss << "Could not find transform between " << robot_frame_id_ << " and " << costmap_sub_->getHeader().frame_id << ": " << ex.what();
      throw nav2_core::PlannerException(ss.str());
    }
  }

  if (optimization_length_ > 0) {
    // find the closest pose on the path
    auto current_pose =
      nav2_util::geometry_utils::min_by(
        path.poses.begin(), path.poses.end(),
        [&robot_pose, this](const geometry_msgs::msg::PoseStamped & ps) {
          return poseDistance(robot_pose, ps);
        });
    
    // expand forwards to extract desired length
    double length = 0;
    end = current_pose - path.poses.begin();
    while ((int)end < (int)path.poses.size()-1 && length < optimization_length_) {
      length += std::hypot(path.poses[end+1].pose.position.x - path.poses[end].pose.position.x,
                           path.poses[end+1].pose.position.y - path.poses[end].pose.position.y);
      end++;
    }
    end++; // end is exclusive

    // expand backwards to extract desired length
    begin = current_pose - path.poses.begin();
    length = 0;
    while (begin > 0 && length < optimization_length_backwards_) {
      length += std::hypot(path.poses[begin+1].pose.position.x - path.poses[begin].pose.position.x,
                           path.poses[begin+1].pose.position.y - path.poses[begin].pose.position.y);
      begin--;
    }
  }
  else {
    begin = 0;
    end = path.poses.size();
  }
}

}  // namespace nav2_smoother
