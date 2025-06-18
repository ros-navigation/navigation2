// Copyright (C) 2025 Pranav Kolekar
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

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <cstdio>
#include <chrono>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav2_toolkit/pose_saver_node.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace nav2_toolkit
{

PoseSaverNode::PoseSaverNode(const rclcpp::NodeOptions & options)
: Node("pose_saver_node", options), saving_active_(false)
{
  this->declare_parameter<double>("save_interval_sec", 1.0);
  this->declare_parameter<std::string>("pose_file_path", 
    std::string(std::getenv("HOME")) + "last_known_pose.yaml");
  this->declare_parameter<bool>("auto_start_saving", true);
  this->declare_parameter<bool>("auto_restore_pose", true);

  pose_file_path_ = this->get_parameter("pose_file_path").as_string();
  double interval_sec = this->get_parameter("save_interval_sec").as_double();
  bool auto_start = this->get_parameter("auto_start_saving").as_bool();
  auto_restore_ = this->get_parameter("auto_restore_pose").as_bool();

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(interval_sec),
    std::bind(&PoseSaverNode::timer_callback, this));
  timer_->cancel();

  sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", 10,
    std::bind(&PoseSaverNode::pose_callback, this, std::placeholders::_1));

  initial_pose_pub_ = this->create_publisher
    <geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
  last_sub_count_ = 0;
  amcl_monitor_timer_ = this->create_wall_timer(
      2s,
      std::bind(&PoseSaverNode::amcl_monitor_callback, this));

  start_service_ = this->create_service<std_srvs::srv::Trigger>(
    "start_pose_saver",
    std::bind(&PoseSaverNode::start_service_cb, this, 
      std::placeholders::_1, std::placeholders::_2));

  stop_service_ = this->create_service<std_srvs::srv::Trigger>(
    "stop_pose_saver",
    std::bind(&PoseSaverNode::stop_service_cb, this, 
      std::placeholders::_1, std::placeholders::_2));

  restore_service_ = this->create_service<std_srvs::srv::Trigger>(
    "localise_at_last_known_position",
    std::bind(&PoseSaverNode::restore_service_cb, this, 
      std::placeholders::_1, std::placeholders::_2));

  if (auto_start) {
    saving_active_ = true;
    timer_->reset();
    RCLCPP_INFO(this->get_logger(), "Pose saving auto-started on launch.");
  }

  if (auto_restore_) {
    RCLCPP_INFO(this->get_logger(), "Auto-restore enabled. Waiting for AMCL to become active...");
    if (this->get_parameter_or("use_sim_time", false)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /clock");
    }
    rclcpp::Time start = this->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(15.0);
    while (initial_pose_pub_->get_subscription_count() == 0 &&
           (this->now() - start) < timeout)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for AMCL");
    }
    if (initial_pose_pub_->get_subscription_count() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Timeout: AMCL did not become active.");
    } else {
      restore_pose_from_file_and_publish();
    }
  }
  RCLCPP_INFO(this->get_logger(), "Pose Saver Node Initialized.");
}

void PoseSaverNode::pose_callback(const 
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  last_pose_ = *msg;
}

void PoseSaverNode::timer_callback()
{
  if (!saving_active_ || !last_pose_) return;

  try {
    write_pose_to_file(pose_file_path_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to save pose: %s", e.what());
  }
}


void PoseSaverNode::start_service_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  saving_active_ = true;
  timer_->reset();
  res->success = true;
  res->message = "Pose saving started.";
}

void PoseSaverNode::stop_service_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  saving_active_ = false;
  timer_->cancel();
  res->success = true;
  res->message = "Pose saving stopped.";
}

void PoseSaverNode::restore_service_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (restore_pose_from_file_and_publish()) {
    res->success = true;
    res->message = "Pose restored from file.";
  } else {
    res->success = false;
    res->message = "Failed to restore pose.";
  }
}

void PoseSaverNode::write_pose_to_file(const std::string &final_path_str)
{
  if (!last_pose_) return;

  YAML::Emitter out;
  out << YAML::BeginMap;

  out << YAML::Key << "header" << YAML::BeginMap;
  out << YAML::Key << "frame_id" << YAML::Value << last_pose_->header.frame_id;
  out << YAML::Key << "stamp" << YAML::BeginMap;
  out << YAML::Key << "sec" << YAML::Value << last_pose_->header.stamp.sec;
  out << YAML::Key << "nanosec" << YAML::Value << last_pose_->header.stamp.nanosec;
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::Key << "pose" << YAML::BeginMap;
  out << YAML::Key << "position" << YAML::BeginMap
      << YAML::Key << "x" << YAML::Value << last_pose_->pose.pose.position.x
      << YAML::Key << "y" << YAML::Value << last_pose_->pose.pose.position.y
      << YAML::Key << "z" << YAML::Value << last_pose_->pose.pose.position.z
      << YAML::EndMap;
  out << YAML::Key << "orientation" << YAML::BeginMap
      << YAML::Key << "x" << YAML::Value << last_pose_->pose.pose.orientation.x
      << YAML::Key << "y" << YAML::Value << last_pose_->pose.pose.orientation.y
      << YAML::Key << "z" << YAML::Value << last_pose_->pose.pose.orientation.z
      << YAML::Key << "w" << YAML::Value << last_pose_->pose.pose.orientation.w
      << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;

  namespace fs = std::filesystem;
  const fs::path final_path(final_path_str);
  const fs::path tmp_path = fs::path("/tmp") / ("pose_saver_" + 
    std::to_string(std::hash<std::string>{}(final_path_str)) + ".tmp");

  std::ofstream tmp_out(tmp_path);
  if (!tmp_out) {
    throw std::runtime_error("Failed to open temporary file: " + tmp_path.string());
  }
  tmp_out << out.c_str();
  tmp_out.close();

  std::error_code ec;
  fs::rename(tmp_path, final_path, ec);
  if (ec) {
    throw std::runtime_error("Failed to rename temporary file: " + ec.message());
  }
  if (ec) {
    throw std::runtime_error("Failed to atomically replace pose file: " + ec.message());
  }
}

void PoseSaverNode::amcl_monitor_callback()
{
  if (!auto_restore_) return;

  int current_count = initial_pose_pub_->get_subscription_count();
  if (last_sub_count_ > 0 && current_count == 0) {
    RCLCPP_WARN(this->get_logger(), "Lost AMCL. Waiting to recover...");
  }
  if (last_sub_count_ == 0 && current_count > 0) {
    RCLCPP_INFO(this->get_logger(), "AMCL reconnected! Publishing last known pose.");
    restore_pose_from_file_and_publish();
  }
  last_sub_count_ = current_count;
}


geometry_msgs::msg::PoseWithCovarianceStamped PoseSaverNode::read_pose_from_file(const std::string &filepath)
{
  YAML::Node node = YAML::LoadFile(filepath);
  geometry_msgs::msg::PoseWithCovarianceStamped msg;

  if (node["header"]) {
    msg.header.frame_id = node["header"]["frame_id"].as<std::string>();
    msg.header.stamp.sec = node["header"]["stamp"]["sec"].as<int32_t>();
    msg.header.stamp.nanosec = node["header"]["stamp"]["nanosec"].as<uint32_t>();
  } else {
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
  }

  msg.pose.pose.position.x = node["pose"]["position"]["x"].as<double>();
  msg.pose.pose.position.y = node["pose"]["position"]["y"].as<double>();
  msg.pose.pose.position.z = node["pose"]["position"]["z"].as<double>();
  msg.pose.pose.orientation.x = node["pose"]["orientation"]["x"].as<double>();
  msg.pose.pose.orientation.y = node["pose"]["orientation"]["y"].as<double>();
  msg.pose.pose.orientation.z = node["pose"]["orientation"]["z"].as<double>();
  msg.pose.pose.orientation.w = node["pose"]["orientation"]["w"].as<double>();

  return msg;
}
bool PoseSaverNode::restore_pose_from_file_and_publish()
{
  try {
    auto pose_msg = read_pose_from_file(pose_file_path_);
    if (initial_pose_pub_->get_subscription_count() == 0) {
      RCLCPP_WARN(this->get_logger(), "No subscribers to /initialpose.");
      return false;
    }

    initial_pose_pub_->publish(pose_msg);
    RCLCPP_INFO(this->get_logger(), "Pose restored from file.");
    return true;
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "Failed to restore pose: %s", e.what());
    return false;
  }
}

}  // namespace nav2_toolkit

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_toolkit::PoseSaverNode)