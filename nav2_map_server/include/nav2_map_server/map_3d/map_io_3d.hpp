// Copyright (c) 2020 Shivam Pandey pandeyshivam2017robotics@gmail.com
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
//
// Created by shivam on 7/5/20.
//

#ifndef NAV2_MAP_SERVER_3D__MAP_IO_3D_HPP_
#define NAV2_MAP_SERVER_3D__MAP_IO_3D_HPP_

#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace nav2_map_server
{

namespace map_3d {
struct Origin {
  std::vector<float> center;
  std::vector<float> orientation;
};

/**
 * @brief parameters that will be populated while reading a YAML file
 */
struct LoadParameters {
  std::string pcd_file_name;
  Origin origin;
};

typedef enum {
  LOAD_MAP_SUCCESS,
  MAP_DOES_NOT_EXIST,
  INVALID_MAP_METADATA,
  INVALID_MAP_DATA
} LOAD_MAP_STATUS;

/**
 * @brief Load and parse the given YAML file
 * @param yaml_filename Name of the map file passed though parameter
 * @return Map loading parameters obtained from YAML file
 * @throw YAML::Exception
 */
LoadParameters loadMapYaml(const std::string &yaml_filename);

/**
 * @brief Load the point cloud from map file and generate a PointCloud2(PCD2)
 * @param load_parameters Parameters of loading map
 * @param map Output loaded map
 * @throw std::exception
 */
void loadMapFromFile(const LoadParameters &load_parameters,
                     sensor_msgs::msg::PointCloud2 &map_msg,
                     geometry_msgs::msg::Pose &origin_msg);

/**
 * @brief Load the map YAML, image from map file and
 * generate a PointCloud2(PCD2)
 * @param yaml_file Name of input YAML file
 * @param map_msg Output loaded map
 * @return status of map loaded
 */
LOAD_MAP_STATUS loadMapFromYaml(const std::string &yaml_file,
                                sensor_msgs::msg::PointCloud2 &map_msg,
                                geometry_msgs::msg::Pose &origin_msg);

struct SaveParameters {
  std::string map_file_name;
  std::string format{"pcd"};
  Origin origin;
  bool as_binary = false;
};

/**
 * @brief Write PointCloud map to file
 * @param map PointCloud2 map data
 * @param save_parameters Map saving parameters.
 * @return true or false
 */
bool saveMapToFile(
    const sensor_msgs::msg::PointCloud2 &map,
    const SaveParameters &save_parameters);

}  // map_3d
}  // namespace nav2_map_server
#endif  // NAV2_MAP_SERVER_3D__MAP_IO_3D_HPP_
