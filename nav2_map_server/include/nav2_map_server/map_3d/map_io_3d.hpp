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

/* PointCloud map input-output library */

#ifndef NAV2_MAP_SERVER__MAP_3D__MAP_IO_3D_HPP_
#define NAV2_MAP_SERVER__MAP_3D__MAP_IO_3D_HPP_

#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace nav2_map_server
{

namespace map_3d
{

/**
 * @brief parameters that will be populated while reading a YAML file
 */
struct LoadParameters
{
  LoadParameters()
  {
    origin.position.x = 0.0;
    origin.position.y = 0.0;
    origin.position.z = 0.0;

    // initialize orientation
    origin.orientation.w = 1.0;
    origin.orientation.x = 0.0;
    origin.orientation.y = 0.0;
    origin.orientation.z = 0.0;
  }

  std::string pcd_file_name;
  geometry_msgs::msg::Pose origin;
  bool use_ext_origin = true;
};

enum class LOAD_MAP_STATUS
{
  LOAD_MAP_SUCCESS,
  MAP_DOES_NOT_EXIST,
  INVALID_MAP_METADATA,
  INVALID_MAP_DATA
};

/**
 * @brief Load and parse the given YAML file
 * @param yaml_filename Name of the map file passed though parameter
 * @return Map loading parameters obtained from YAML file
 * @throw YAML::Exception
 */
LoadParameters loadMapYaml(const std::string & yaml_filename);

/**
 * @brief Load the point cloud from map file and generate a PointCloud2(PCD2)
 * @param load_parameters Parameters of loading map
 * @param map Output loaded map
 * @throw std::exception
 */
void loadMapFromFile(
  const LoadParameters & load_parameters,
  sensor_msgs::msg::PointCloud2 & map_msg,
  geometry_msgs::msg::Pose & origin_msg);

/**
 * @brief Load the map YAML, pcd from map file and
 * generate a PointCloud2(PCD2)
 * @param yaml_file Name of input YAML file
 * @param map_msg Output loaded map
 * @param origin_msg Output loaded viewpoint
 * @return status of map loaded
 */
LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  sensor_msgs::msg::PointCloud2 & map_msg,
  geometry_msgs::msg::Pose & origin_msg);

/**
 * @brief SaveParameters for 3D map usage
 */
struct SaveParameters
{
  SaveParameters()
  {
    origin.position.x = 0.0;
    origin.position.y = 0.0;
    origin.position.z = 0.0;

    // initialize orientation
    origin.orientation.w = 1.0;
    origin.orientation.x = 0.0;
    origin.orientation.y = 0.0;
    origin.orientation.z = 0.0;
  }

  std::string map_file_name{""};
  std::string format{"pcd"};
  geometry_msgs::msg::Pose origin;
  bool as_binary = false;
};

/**
 * @brief Write PointCloud map to file
 * @param map PointCloud2 map data
 * @param save_parameters Map saving parameters.
 * @return true or false
 */
bool saveMapToFile(
  const sensor_msgs::msg::PointCloud2 & map,
  const SaveParameters & save_parameters);

}  // namespace map_3d

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_3D__MAP_IO_3D_HPP_
