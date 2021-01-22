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

#include "map_mode.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_map_server
{
class OccGridLoader : public nav2_util::LifecycleHelperInterface
{
public:
  OccGridLoader(rclcpp_lifecycle::LifecycleNode::SharedPtr node
    , std::string & yaml_filename
    , std::string & topic_name
    , std::string & frame_id);
  OccGridLoader() = delete;
  ~OccGridLoader();

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

protected:
  // The ROS node to use for ROS-related operations such as creating a service
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // The name of the YAML file from which to get the conversion parameters
  std::string yaml_filename_;
  std::string topic_name_;
  std::string frame_id_;

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

  // Load and parse the given YAML file
  /// @throw YAML::Exception
  LoadParameters load_map_yaml(const std::string & yaml_filename_);

  // Load the image and generate an OccupancyGrid
  void loadMapFromFile(const LoadParameters & loadParameters);

  // A service to provide the occupancy grid (GetMap) and the message to return
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr occ_service_;

  // A topic on which the occupancy grid will be published
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;

  // The message to publish on the occupancy grid topic
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> msg_;

  // The name of the service for getting a map
  static constexpr const char * service_name_{"map"};

  // Timer for republishing map
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__OCC_GRID_LOADER_HPP_
