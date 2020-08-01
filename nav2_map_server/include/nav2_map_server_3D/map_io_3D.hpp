//
// Created by shivam on 7/5/20.
//

#ifndef NAV2_MAP_SERVER_SRC_MAP_IO_3D_HPP_
#define NAV2_MAP_SERVER_SRC_MAP_IO_3D_HPP_

#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform.hpp"

namespace nav2_map_server {

/**
 * @brief nav2_map_server_3D namespace containing the utilities to use point cloud mapss
 */
namespace nav2_map_server_3D {

/**
 * @brief parameters that will be populated while reading a YAML file
 */
struct LoadParameters_3D {
  std::string pcd_file_name;
  std::vector<float> view_point;
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
LoadParameters_3D loadMapYaml(const std::string &yaml_filename);

/**
 * @brief Load the point cloud from map file and generate an OccupancyGrid
 * @param load_parameters Parameters of loading map
 * @param map Output loaded map
 * @throw std::exception
 */
void loadMapFromFile(
  const LoadParameters_3D &load_parameters,
  sensor_msgs::msg::PointCloud2 &map,
  geometry_msgs::msg::Transform &view_point_msg);

/**
 * @brief Load the map YAML, image from map file and
 * generate an OccupancyGrid
 * @param yaml_file Name of input YAML file
 * @param map Output loaded map
 * @return status of map loaded
 */
LOAD_MAP_STATUS loadMapFromYaml(
  const std::string &yaml_file,
  sensor_msgs::msg::PointCloud2 &map,
  geometry_msgs::msg::Transform &view_point_msg);

struct SaveParameters {
  std::string map_file_name;
  std::vector<float> view_point;
  bool as_binary = false;
  std::string format{"pcd"};
};

/**
 * @brief Write OccupancyGrid map to file
 * @param map OccupancyGrid map data
 * @param save_parameters Map saving parameters.
 * @return true or false
 */
bool saveMapToFile(
  const sensor_msgs::msg::PointCloud2 &map,
  const SaveParameters &save_parameters);

} // namespace nav2_map_server_3D
} // namespace nav2_map_server
#endif //NAV2_MAP_SERVER_SRC_MAP_IO_3D_HPP_
