/* Copyright (c) 2020 Shivam Pandey
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "nav2_map_server/map_3d/map_saver_3d.hpp"

#include <string>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <functional>

#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::placeholders;

namespace nav2_map_server
{

MapSaver3D::MapSaver3D()
: nav2_util::LifecycleNode("map_saver", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");

  save_map_timeout_ = std::make_shared<rclcpp::Duration>(
    rclcpp::Duration::from_seconds(declare_parameter("save_map_timeout", 2.0)));

  map_subscribe_transient_local_ = declare_parameter("map_subscribe_transient_local", true);
}

MapSaver3D::~MapSaver3D()
{
}

nav2_util::CallbackReturn
MapSaver3D::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Make name prefix for services
  const std::string service_prefix = get_name() + std::string("/");

  // Create a service that saves the occupancy grid or PointCloud2 from map topic to a file
  save_map_service_ = create_service<nav2_msgs::srv::SaveMap3D>(
    service_prefix + save_map_service_name_,
    std::bind(&MapSaver3D::saveMapCallback, this, _1, _2, _3));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapSaver3D::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapSaver3D::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapSaver3D::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  save_map_service_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapSaver3D::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void MapSaver3D::saveMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::SaveMap3D::Request> request,
  std::shared_ptr<nav2_msgs::srv::SaveMap3D::Response> response)
{
  map_3d::SaveParameters save_parameters;
  save_parameters.map_file_name = request->map_url;
  save_parameters.as_binary = request->as_binary;
  save_parameters.format = request->file_format;

  response->result = saveMapTopicToFile(request->map_topic, save_parameters);
}

bool MapSaver3D::saveMapTopicToFile(
  const std::string & map_topic,
  const map_3d::SaveParameters & save_parameters)
{
  // Local copies of map_topic and save_parameters that could be changed
  std::string map_topic_loc = map_topic;
  map_3d::SaveParameters save_parameters_loc = save_parameters;

  RCLCPP_INFO(
    get_logger(), "Saving map from \'%s\' topic and default origin to \'%s\' file",
    map_topic_loc.c_str(), save_parameters_loc.map_file_name.c_str());

  try {
    // Pointer to map message received in the subscription callback
    sensor_msgs::msg::PointCloud2::SharedPtr pcd_map_msg = nullptr;

    // Mutex for handling map_msg shared resource
    std::recursive_mutex access;

    // Correct map_topic_loc if necessary
    if (map_topic_loc.empty()) {
      map_topic_loc = "map";
      RCLCPP_WARN(
        get_logger(), "Map topic unspecified. Map messages will be read from \'%s\' topic",
        map_topic_loc.c_str());
    }

    // A callback function that receives map message from subscribed topic
    auto map_callback = [&pcd_map_msg, &access](
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
        std::lock_guard<std::recursive_mutex> guard(access);
        pcd_map_msg = msg;
      };

    // Add new subscription for incoming map topic.
    // Utilizing local rclcpp::Node (rclcpp_node_) from nav2_util::LifecycleNode
    // as a map listener.
    rclcpp::QoS map_qos(10);  // initialize to default
    if (map_subscribe_transient_local_) {
      map_qos.transient_local();
      map_qos.reliable();
      map_qos.keep_last(1);
    }
    auto pcd_map_sub = rclcpp_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      map_topic_loc, map_qos, map_callback);

    rclcpp::Time start_time = now();
    while (rclcpp::ok()) {
      if ((now() - start_time) > *save_map_timeout_) {
        RCLCPP_ERROR(get_logger(), "Failed to save the map: timeout");
        return false;
      }

      if (pcd_map_msg) {
        std::lock_guard<std::recursive_mutex> guard(access);
        // map_sub is no more needed
        pcd_map_sub.reset();

        // Map message received. Saving it to file
        if (map_3d::saveMapToFile(*pcd_map_msg, save_parameters_loc)) {
          RCLCPP_INFO(get_logger(), "Map saved successfully");
          return true;
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to save the map");
          return false;
        }
      }

      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to save the map: %s", e.what());
    return false;
  }

  return false;
}

}  // namespace nav2_map_server
