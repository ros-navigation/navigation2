//
// Created by shivam on 9/15/20.
//

#include <string>
#include <memory>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "map_3d/map_3d.hpp"
#include "map_3d/map_io_3d.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav2_map_server {

MapServer<sensor_msgs::msg::PointCloud2>::MapServer()
    : nav2_util::LifecycleNode("map_server") {
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare the node parameters
  declare_parameter("yaml_filename");
  declare_parameter("topic_name", "map");
  declare_parameter("frame_id", "map");
}

MapServer<sensor_msgs::msg::PointCloud2>::~MapServer() {
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn MapServer<sensor_msgs::msg::PointCloud2>
::on_configure(const rclcpp_lifecycle::State &state) {

  RCLCPP_INFO(get_logger(), "Configuring");

  // Get the name of the YAML file to use
  std::string yaml_filename = get_parameter("yaml_filename").as_string();

  std::string topic_name = get_parameter("topic_name").as_string();
//  frame_id_ = get_parameter("frame_id").as_string();

  // Shared pointer to LoadMap::Response is also should be initialized
  // in order to avoid null-pointer dereference
  std::shared_ptr<nav2_msgs::srv::LoadMap3D::Response> rsp =
      std::make_shared<nav2_msgs::srv::LoadMap3D::Response>();

  if (!loadMapResponseFromYaml(yaml_filename, rsp)) {
    throw std::runtime_error("Failed to load map yaml file: " + yaml_filename);
  }

  // Make name prefix for services
  const std::string service_prefix = get_name() + std::string("/");

  // Create a service that provides the PointCloud2
  auto get_map_callback_lambda = [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<nav2_msgs::srv::GetMap3D::Request> request,
      std::shared_ptr<nav2_msgs::srv::GetMap3D::Response> response) {
    getMapCallback(request_header, request, response);
  };

  pcd_service_ = create_service<nav2_msgs::srv::GetMap3D>(
      service_prefix + std::string(service_name_),
      get_map_callback_lambda);

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  pcd_pub_ = create_publisher<nav2_msgs::msg::PCD2>(
      topic_name,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Create a service that loads the PointCloud2 from a file
  auto load_map_callback_lambda = [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<nav2_msgs::srv::LoadMap3D::Request> request,
      std::shared_ptr<nav2_msgs::srv::LoadMap3D::Response> response) {
    loadMapCallback(request_header, request, response);
  };

  pcd_load_map_service_ = create_service<nav2_msgs::srv::LoadMap3D>(
      service_prefix + std::string(load_map_service_name_), load_map_callback_lambda);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapServer<sensor_msgs::msg::PointCloud2>
::on_activate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Activating");

  // Publish the map(pcd if enabled) using the latched topic
  pcd_pub_->on_activate();
  auto pcd_cloud_2 = std::make_unique<nav2_msgs::msg::PCD2>(pcd_msg_);
  pcd_pub_->publish(std::move(pcd_cloud_2));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapServer<sensor_msgs::msg::PointCloud2>
::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pcd_pub_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapServer<sensor_msgs::msg::PointCloud2>
::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  pcd_pub_.reset();
  pcd_service_.reset();
  pcd_load_map_service_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer<sensor_msgs::msg::PointCloud2>::on_error(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer<sensor_msgs::msg::PointCloud2>::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool MapServer<sensor_msgs::msg::PointCloud2>
::loadMapResponseFromYaml(const std::string &yaml_file,
                          std::shared_ptr<nav2_msgs::srv::LoadMap3D::Response> response) {
  switch (map_3d::loadMapFromYaml(yaml_file, pcd_msg_)) {
    case map_3d::MAP_DOES_NOT_EXIST:response->result = nav2_msgs::srv::LoadMap3D::Response::RESULT_MAP_DOES_NOT_EXIST;
      return false;
    case map_3d::INVALID_MAP_METADATA:
      response->result = nav2_msgs::srv::LoadMap3D::Response::RESULT_INVALID_MAP_METADATA;
      return false;
    case map_3d::INVALID_MAP_DATA:response->result = nav2_msgs::srv::LoadMap3D::Response::RESULT_INVALID_MAP_DATA;
      return false;
    case map_3d::LOAD_MAP_SUCCESS: response->map = pcd_msg_.map;
      response->origin = pcd_msg_.origin;
      response->orientation = pcd_msg_.orientation;
      response->result = nav2_msgs::srv::LoadMap3D::Response::RESULT_SUCCESS;
  }

  return true;
}

void MapServer<sensor_msgs::msg::PointCloud2>
::getMapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                 const std::shared_ptr<nav2_msgs::srv::GetMap3D::Request> request,
                 std::shared_ptr<nav2_msgs::srv::GetMap3D::Response> response) {
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
        get_logger(),
        "Received GetMap request but not in ACTIVE state, ignoring!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling GetMap request");
  response->map = pcd_msg_.map;
  response->origin = pcd_msg_.origin;
  response->orientation = pcd_msg_.orientation;
}

void MapServer<sensor_msgs::msg::PointCloud2>
::loadMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::LoadMap3D::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap3D::Response> response) {
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
        get_logger(),
        "Received LoadMap request but not in ACTIVE state, ignoring!");
    return;
  }

  RCLCPP_INFO(get_logger(), "Handling LoadMap request");

  // Load from file
  if (loadMapResponseFromYaml(request->map_url, response)) {
    auto pcd_msg = std::make_unique<nav2_msgs::msg::PCD2>(pcd_msg_);
    pcd_pub_->publish(std::move(pcd_msg));  // publish new map
  }
}

}  // namespace nav2_map_server