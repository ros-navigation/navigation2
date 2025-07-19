// Copyright (c) 2025 Pranav Kolekar
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
#include <chrono>
#include <fstream>
#include <filesystem>
#include <memory>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "nav2_toolkit/pose_saver_node.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace nav2_toolkit
{

PoseSaverNode::PoseSaverNode(const rclcpp::NodeOptions & options)
: Node("pose_saver_node", options),
  pose_restored_(false),
  was_pose_pub_up_last_check_(false)
{
  this->declare_parameter("save_interval_sec", 5.0);
  this->declare_parameter("pose_file_path",
      std::string(std::getenv("HOME")) + "/last_known_pose.yaml");
  this->declare_parameter("auto_start_saving", true);
  this->declare_parameter("auto_restore_pose", true);

  pose_file_path_ = this->get_parameter("pose_file_path").as_string();
  double interval_sec = this->get_parameter("save_interval_sec").as_double();
  bool auto_start = this->get_parameter("auto_start_saving").as_bool();
  auto_restore_ = this->get_parameter("auto_restore_pose").as_bool();

  sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", 10,
    std::bind(&PoseSaverNode::pose_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(interval_sec),
    std::bind(&PoseSaverNode::timer_callback, this));
  timer_->cancel();

  pose_publisher_monitor_timer_ = this->create_wall_timer(
    2s,
    std::bind(&PoseSaverNode::pose_publisher_monitor_callback, this));

  start_service_ = this->create_service<std_srvs::srv::Trigger>(
    "start_pose_saver",
    std::bind(&PoseSaverNode::start_service_cb, this,
       std::placeholders::_1, std::placeholders::_2));

  stop_service_ = this->create_service<std_srvs::srv::Trigger>(
    "stop_pose_saver",
    std::bind(&PoseSaverNode::stop_service_cb, this, std::placeholders::_1, std::placeholders::_2));

  restore_service_ = this->create_service<std_srvs::srv::Trigger>(
    "localise_at_last_known_position",
    std::bind(&PoseSaverNode::restore_service_cb, this,
      std::placeholders::_1, std::placeholders::_2));

  if (auto_start) {
    timer_->reset();
    RCLCPP_INFO(this->get_logger(), "Pose saving auto-started on launch.");
  }

  if (auto_restore_) {
    RCLCPP_INFO(this->get_logger(), "Auto-restore enabled. Waiting for pose publisher...");

    if (this->get_parameter_or("use_sim_time", false)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /clock to start (sim time)...");
      rclcpp::Rate rate(10);
      while (rclcpp::ok() && this->now().nanoseconds() == 0) {
        rate.sleep();
      }
    }
  }

  post_init_timer_ = this->create_wall_timer(
    100ms,
    std::bind(&PoseSaverNode::post_init_setup, this));

  RCLCPP_INFO(this->get_logger(), "Pose Saver Node Initialized.");
}

void PoseSaverNode::post_init_setup()
{
  post_init_timer_->cancel();
  try {
    set_pose_client_ = this->create_client<nav2_msgs::srv::SetInitialPose>(
      "set_initial_pose",
      false  // Do not spin internal executor
    );


    RCLCPP_INFO(this->get_logger(), "Service client created and ready.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize service client: %s", e.what());
  }
}

void PoseSaverNode::pose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  last_pose_ = msg;
}

void PoseSaverNode::timer_callback()
{
  if (!last_pose_) {return;}

  try {
    write_pose_to_file(pose_file_path_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to save pose: %s", e.what());
  }
}

void PoseSaverNode::start_service_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  timer_->reset();
  res->success = true;
  res->message = "Pose saving started.";
}

void PoseSaverNode::stop_service_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
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

void PoseSaverNode::pose_publisher_monitor_callback()
{
  if (!auto_restore_) {
    return;
  }

  const bool pose_publisher_up = set_pose_client_->wait_for_service(100ms);

  if (!pose_publisher_up) {
    if (was_pose_pub_up_last_check_) {
      RCLCPP_WARN(this->get_logger(), "Lost pose publisher. Waiting to recover...");
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Auto-restore enabled. Waiting for pose publisher...");
    }
    was_pose_pub_up_last_check_ = false;
    pose_restored_ = false;
    return;
  }

  if (!was_pose_pub_up_last_check_) {
    RCLCPP_INFO(this->get_logger(), "pose publisher came online.");
  }

  was_pose_pub_up_last_check_ = true;

  if (!pose_restored_) {
    if (restore_pose_from_file_and_publish()) {
      pose_restored_ = true;
      RCLCPP_INFO(this->get_logger(), "Pose successfully restored.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Pose restore failed. Will retry.");
    }
  }
}

void PoseSaverNode::write_pose_to_file(const std::string & path)
{
  if (!last_pose_) {return;}

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "header" << YAML::BeginMap
      << YAML::Key << "frame_id" << YAML::Value << last_pose_->header.frame_id
      << YAML::Key << "stamp" << YAML::BeginMap
      << YAML::Key << "sec" << YAML::Value << last_pose_->header.stamp.sec
      << YAML::Key << "nanosec" << YAML::Value << last_pose_->header.stamp.nanosec
      << YAML::EndMap << YAML::EndMap;

  out << YAML::Key << "pose" << YAML::BeginMap
      << YAML::Key << "position" << YAML::BeginMap
      << YAML::Key << "x" << YAML::Value << last_pose_->pose.pose.position.x
      << YAML::Key << "y" << YAML::Value << last_pose_->pose.pose.position.y
      << YAML::Key << "z" << YAML::Value << last_pose_->pose.pose.position.z
      << YAML::EndMap;

  out << YAML::Key << "orientation" << YAML::BeginMap
      << YAML::Key << "x" << YAML::Value << last_pose_->pose.pose.orientation.x
      << YAML::Key << "y" << YAML::Value << last_pose_->pose.pose.orientation.y
      << YAML::Key << "z" << YAML::Value << last_pose_->pose.pose.orientation.z
      << YAML::Key << "w" << YAML::Value << last_pose_->pose.pose.orientation.w
      << YAML::EndMap << YAML::EndMap << YAML::EndMap;

  const fs::path final_path(path);
  const fs::path tmp_path = fs::temp_directory_path() /
    ("pose_saver_" + std::to_string(std::hash<std::string>{}(path)) + ".tmp");

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
}

geometry_msgs::msg::PoseWithCovarianceStamped PoseSaverNode::read_pose_from_file(
  const std::string & path)
{
  YAML::Node node = YAML::LoadFile(path);
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
  if (pose_restored_) {
    RCLCPP_INFO(this->get_logger(), "Pose already restored. Skipping.");
    return true;
  }

  auto request = std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
  request->pose = read_pose_from_file(pose_file_path_);

  if (!set_pose_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Service /set_initial_pose not available after waiting");
    return false;
  }

  set_pose_client_->async_send_request(request,
    [this](rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedFuture future)
    {
      if (future.valid()) {
        RCLCPP_INFO(this->get_logger(), "Initial pose successfully restored.");
        pose_restored_ = true;
      } else {
        RCLCPP_WARN(this->get_logger(), "Initial pose service call failed.");
      }
    });

  // Indicate that we *attempted* to restore — actual result is handled in callback
  return true;
}


}  // namespace nav2_toolkit

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_toolkit::PoseSaverNode)
