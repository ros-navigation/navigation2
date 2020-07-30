//
// Created by shivam on 7/5/20.
//

#ifndef NAV2_MAP_SERVER_SRC_MAP_IO_3D_HPP_
#define NAV2_MAP_SERVER_SRC_MAP_IO_3D_HPP_

#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/msg/pcd2.hpp"

namespace nav2_map_server
{
/**
 * @brief nav2_map_server_3D namespace containing the utilities to use point cloud mapss
 */
namespace nav2_map_server_3D
{
/**
 * @brief parameters that will be populated while reading a YAML file
 */
struct LoadParameters_3D {
  std::string pcd_file_name;
  std::vector<float> origin;
  std::vector<float> orientation;
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
LoadParameters_3D loadMapYaml(const std::string & yaml_filename);

/**
 * @brief Load the point cloud from map file and generate a PointCloud2(PCD2)
 * @param load_parameters Parameters of loading map
 * @param map Output loaded map
 * @throw std::exception
 */
void loadMapFromFile(
  const LoadParameters_3D & load_parameters,
  nav2_msgs::msg::PCD2 & map_msg);

/**
 * @brief Load the map YAML, image from map file and
 * generate a PointCloud2(PCD2)
 * @param yaml_file Name of input YAML file
 * @param map_msg Output loaded map
 * @return status of map loaded
 */
LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  nav2_msgs::msg::PCD2 & map_msg);

struct SaveParameters {
  std::string map_file_name;
  std::vector<float> origin;
  std::vector<float> orientation;
  bool as_binary = false;
  std::string format{"pcd"};
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

} // namespace nav2_map_server_3D
} // namespace nav2_map_server
#endif //NAV2_MAP_SERVER_SRC_MAP_IO_3D_HPP_
