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
#include <future>
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

  // Create a service that saves the PointCloud2 from map topic to a file
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
    get_logger(), "Saving map from topic: \'%s\', and default origin [0,0,0] to \'%s\' file",
    map_topic_loc.c_str(), save_parameters_loc.map_file_name.c_str());

  try {
    // Correct map_topic_loc if necessary
    if (map_topic_loc.empty()) {
      map_topic_loc = "map";
      RCLCPP_WARN(
        get_logger(), "Map topic unspecified. Map messages will be read from \'%s\' topic",
        map_topic_loc.c_str());
    }

    std::promise<sensor_msgs::msg::PointCloud2::SharedPtr> prom;
    std::future<sensor_msgs::msg::PointCloud2::SharedPtr> future_result = prom.get_future();

    // A callback function that receives map message from subscribed topic
    auto mapCallback = [&prom](
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
        prom.set_value(msg);
      };

    rclcpp::QoS map_qos(10);  // initialize to default
    if (map_subscribe_transient_local_) {
      map_qos.transient_local();
      map_qos.reliable();
      map_qos.keep_last(1);
    }

    // Create new CallbackGroup for map_sub
    auto callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);

    auto option = rclcpp::SubscriptionOptions();
    option.callback_group = callback_group;
    
    auto map_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      map_topic_loc, map_qos, mapCallback, option);

    // Create SingleThreadedExecutor to spin map_sub in callback_group
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(callback_group, get_node_base_interface());

    // Spin until map message received
    auto timeout = save_map_timeout_->to_chrono<std::chrono::nanoseconds>();
    auto status = executor.spin_until_future_complete(future_result, timeout);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to spin map subscription");
      return false;
    }
    // map_sub is no more needed
    map_sub.reset();
    // Map message received. Saving it to file
    sensor_msgs::msg::PointCloud2::SharedPtr map_msg = future_result.get();
    if (saveMapToFile(*map_msg, save_parameters_loc)) {
      RCLCPP_INFO(get_logger(), "Map saved successfully");
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to save the map");
      return false;
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to save the map: %s", e.what());
    return false;
  }

  return false;
}

}  // namespace nav2_map_server
