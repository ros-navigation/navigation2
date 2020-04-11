// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_MAP_SERVER__OCC_GRID_LOADER_HPP_
#define NAV2_MAP_SERVER__OCC_GRID_LOADER_HPP_

#include <memory>
#include <string>
#include <vector>
#include "nav2_map_server/occ_grid_loader.hpp"
#include "nav2_map_server/visibility_control.h"

#include "map_mode.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_map_server
{
/**
 * @class nav2_map_server::OccGridLoader
 * @brief Parses the map yaml file and creates a service and a publisher that
 * provides occupancy grid
 */
class
NAV2_MAP_SERVER_PUBLIC
OccGridLoader : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  /**
   * @brief Constructor for OccGridLoader
   * @param node
   * @param Yaml_filename File that contains map data
   */
  OccGridLoader(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string & yaml_filename);
  /**
   * @brief Disabling the use of default or empty constructor
   */
  OccGridLoader() = delete;
  /**
   * @brief Destructor for OccGridLoader
   */
  ~OccGridLoader();
  /**
   * @brief Load map and its parameters from the file
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Start publishing the map using the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Stops publishing the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets the member variables
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

protected:
  // The ROS node to use for ROS-related operations such as creating a service
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // The name of the YAML file from which to get the conversion parameters
  std::string yaml_filename_;

  typedef struct
  {
    std::string image_file_name;
    double resolution{0};
    std::vector<double> origin{0, 0, 0};
    double free_thresh;
    double occupied_thresh;
    MapMode mode;
    bool negate;
  } LoadParameters;

  /**
   * @brief Load and parse the given YAML file
   * @param yaml_filename_ Name of the map file passed though parameter
   * @throw YAML::Exception
   */
  LoadParameters load_map_yaml(const std::string & yaml_filename_);

  // Load the image and generate an OccupancyGrid
  void loadMapFromFile(const LoadParameters & loadParameters);

  // Load the map yaml and image from yaml file name
  bool loadMapFromYaml(
    std::string yaml_file,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response = nullptr);

  // A service to provide the occupancy grid (GetMap) and the message to return
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr occ_service_;

  // A service to load the occupancy grid from file at run time (LoadMap)
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_map_service_;

  // A topic on which the occupancy grid will be published
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;

  // The message to publish on the occupancy grid topic
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> msg_;

  // The frame ID used in the returned OccupancyGrid message
  static constexpr const char * frame_id_{"map"};

  // The name for the topic on which the map will be published
  static constexpr const char * topic_name_{"map"};

  // The name of the service for getting a map
  static constexpr const char * service_name_{"map"};

  // The name of the service for loading a map
  static constexpr const char * load_map_service_name_{"load_map"};

  // Timer for republishing map
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__OCC_GRID_LOADER_HPP_
