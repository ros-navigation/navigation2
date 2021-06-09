// Copyright (c) 2020 Shivam Pandey
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

/*Map-io library for 3D pointcloud implementation*/

#include "nav2_map_server/map_3d/map_io_3d.hpp"

#ifndef _WIN32
#include <libgen.h>
#endif
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>

#include "yaml-cpp/yaml.h"
#include "nav2_map_server/map_3d/pcl_helper.hpp"

#include "pcl/io/pcd_io.h"
#include "Eigen/Core"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Scalar.h"

#include "pcl_ros/transforms.hpp"

#ifdef _WIN32
// https://github.com/rtv/Stage/blob/master/replace/dirname.c
static
char * dirname(char * path)
{
  static const char dot[] = ".";
  char * last_slash;

  if (path == NULL) {
    return path;
  }

  /* Find last '/'.  */
  last_slash = path != NULL ? strrchr(path, '/') : NULL;

  if (last_slash != NULL && last_slash == path) {
    /* The last slash is the first character in the string.  We have to
       return "/".  */
    ++last_slash;
  } else if (last_slash != NULL && last_slash[1] == '\0') {
    /* The '/' is the last character, we have to look further.  */
    last_slash = reinterpret_cast<char *>(memchr(path, last_slash - path, '/'));
  }

  if (last_slash != NULL) {
    /* Terminate the path.  */
    last_slash[0] = '\0';
  } else {
    /* This assignment is ill-designed but the XPG specs require to
       return a string containing "." in any case no directory part is
       found and so a static and constant string is required.  */
    path = reinterpret_cast<char *>(dot);
  }

  return path;
}
#endif

namespace nav2_map_server
{

namespace map_3d
{

// === Map input part ===

/// Get the given subnode value.
/// The only reason this function exists is to wrap the exceptions in slightly nicer error messages,
/// including the name of the failed key
/// @throw YAML::Exception
template<typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}

LoadParameters loadMapYaml(const std::string & yaml_filename)
{
  std::cout << "loading the yaml file";

  YAML::Node doc = YAML::LoadFile(yaml_filename);
  LoadParameters load_parameters;

  std::string pcd_file_name(yaml_get_value<std::string>(doc, "pcd"));
  if (!ends_with(pcd_file_name, ".pcd")) {
    throw YAML::Exception(
            doc["image"].Mark(),
            "The pcd file is invalid please pass a file name with extension .pcd");
  }

  if (pcd_file_name[0] != '/') {
    // dirname takes a mutable char *, so we copy into a vector
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    load_parameters.pcd_file_name =
      std::string(dirname(fname_copy.data())) + '/' + pcd_file_name;
  } else {
    load_parameters.pcd_file_name = pcd_file_name;
  }

  // Get view point as position and orientation
  // view point will be loaded from pcd file while reading
  // If not provided by YAML, if it provided by both YAML and PCD then
  // a warning will be issued if they are not equal and PCD  will take precedence
  try {
    // Load origin form yaml_file
    std::vector<double> pcd_origin = yaml_get_value<std::vector<double>>(doc, "origin");
    if (pcd_origin.size() != 7) {
      throw std::invalid_argument("view_point is wrong");
    }

    // Position
    tf2::Vector3 translation = tf2::Vector3(
      tf2Scalar(pcd_origin[0]),
      tf2Scalar(pcd_origin[1]),
      tf2Scalar(pcd_origin[2]));
    // Orientation
    tf2::Quaternion rotation = tf2::Quaternion(
      tf2Scalar(pcd_origin[4]),
      tf2Scalar(pcd_origin[5]),
      tf2Scalar(pcd_origin[6]),
      tf2Scalar(pcd_origin[3]));
    // Transform
    load_parameters.origin = tf2::Transform(rotation, translation);
  } catch (YAML::Exception & e) {
    std::cout << "[WARNING] [map_io_3d]: Couldn't load view_point from yaml file: " << std::endl;
  }

  return load_parameters;
}

void loadMapFromFile(
  const LoadParameters & load_parameters,
  sensor_msgs::msg::PointCloud2 & map_msg)
{
  sensor_msgs::msg::PointCloud2 map, map2;

  std::cout << "[INFO] [map_io_3d]: Loading pcd_file: " <<
    load_parameters.pcd_file_name << std::endl;

  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());

  //  create a pcd reader for PointCloud2 data
  pcl::PCDReader reader;

  Eigen::Vector4f position;
  Eigen::Quaternionf orientation;

  // PCD version fixed to version 0.7
  // will use sensor origin (added in PCD version 0.7)
  int pcd_version = pcl::PCDReader::PCD_V7;

  std::cout << "[DEBUG] [map_io_3d]: feeding pcd filename to reader" << std::endl;

  if (reader.read(
      load_parameters.pcd_file_name, *cloud,
      position, orientation, pcd_version) == -1)              //* load the file
  {
    std::string error_msg{"Couldn't read "};
    error_msg += load_parameters.pcd_file_name + "\n";
    PCL_ERROR(error_msg.c_str());
  }
  std::cout << "[DEBUG] [map_io_3d]: pcd file is loaded" << std::endl;

  std::cout << "[DEBUG] [map_io_3d]: converting pcd to message" << std::endl;

  //  update message data
  pclToMsg(map, cloud);

  std::cout << "[DEBUG] [map_io_3d]: message conversion is done" << std::endl;

  pcl_ros::transformPointCloud("random", load_parameters.origin, map, map2);
  map_msg = map2;
}

LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  sensor_msgs::msg::PointCloud2 & map_msg)
{
  if (yaml_file.empty()) {
    std::cerr << "[ERROR] [map_io_3d]: YAML file name is empty, can't load!" << std::endl;
    return LOAD_MAP_STATUS::MAP_DOES_NOT_EXIST;
  }

  std::cout << "[INFO] [map_io_3d]: Loading yaml file: " << yaml_file << std::endl;
  LoadParameters load_parameters;

  try {
    std::cout << "initiating load_parameters";

    load_parameters = loadMapYaml(yaml_file);
  } catch (YAML::Exception & e) {
    std::cerr << "[ERROR] [map_io_3d]: Failed processing YAML file " << yaml_file <<
      " at position (" << e.mark.line << ":" << e.mark.column << ") for reason: " <<
      e.what() << std::endl;

    return LOAD_MAP_STATUS::INVALID_MAP_METADATA;
  }

  try {
    loadMapFromFile(load_parameters, map_msg);
  } catch (std::exception & e) {
    std::cerr <<
      "[ERROR] [map_io_3d]: Failed to load image file " << load_parameters.pcd_file_name <<
      " for reason: " << e.what() << std::endl;

    return LOAD_MAP_STATUS::INVALID_MAP_DATA;
  }

  return LOAD_MAP_STATUS::LOAD_MAP_SUCCESS;
}

// === Map output part ===

/**
 * @brief Checks map saving parameters for consistency
 * @param save_parameters Map saving parameters.
 * NOTE: save_parameters could be updated during function execution.
 * @throw std::exception in case of inconsistent parameters
 */
void checkSaveParameters(SaveParameters & save_parameters)
{
  // Check for empty file name/path
  if (save_parameters.map_file_name.empty()) {
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    save_parameters.map_file_name =
      "map_" + std::to_string(static_cast<int>(clock.now().seconds()));
    std::cout << "[WARNING] [map_io_3d]: Map file unspecified. Map will be saved to " <<
      save_parameters.map_file_name << " file" << std::endl;
  }

  // Check for encoding
  if (save_parameters.as_binary) {
    std::cout << "[WARNING] [map_io_3d]: Map will be saved in binary form to " <<
      save_parameters.map_file_name << " file" << std::endl;
  }

  // Check for file format
  if (save_parameters.format.empty()) {
    save_parameters.format = "pcd";
    std::cout << "[WARNING] [map_io_3d]: No map format is "
      "specified. pcd format will be used." << std::endl;
  }

  // Confirm for the presently implemented formats
  if (save_parameters.format != "ply" && save_parameters.format != "pcd") {
    save_parameters.format = "pcd";
    std::cout << "[WARNING] [map_io_3d]: " << save_parameters.format <<
      " support is not implemented, Falling back to pcd file format" << std::endl;
  }

  if (save_parameters.format == "ply") {
    // TODO(Shivam Pandey): add ply support
    save_parameters.format = "pcd";
    std::cout << "[WARNING] [map_io_3d]: ply support is not implemented, "
      "Falling back to pcd file format" << std::endl;
  }
}

/**
 * @brief Tries to write map data into a file
 * @param map Pointcloud2 data
 * @param save_parameters Map saving parameters
 * @throw std::exception in case of problem
 */
void tryWriteMapToFile(
  const sensor_msgs::msg::PointCloud2 & map,
  const SaveParameters & save_parameters)
{
  std::string file_name(save_parameters.map_file_name);

  if (save_parameters.format == "pcd") {
    file_name += ".pcd";
  } else if (save_parameters.format == "ply") {
    file_name += ".ply";
  }

  pcl::PCLPointCloud2::Ptr cloud_2(new pcl::PCLPointCloud2());
  msgToPcl(cloud_2, map);

  pcl::PCDWriter writer;

  // Initialize origin
  Eigen::Vector4f position = Eigen::Vector4f::Zero();
  // Initialize orientation
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();

  if (writer.write(
      file_name, cloud_2, position,
      orientation, save_parameters.as_binary) == -1)
  {
    std::string error_msg{"Couldn't write "};
    error_msg += file_name + "\n";
    PCL_ERROR(error_msg.c_str());
  }

  std::string mapmetadatafile = save_parameters.map_file_name + ".yaml";
  {
    std::ofstream yaml(mapmetadatafile);

    YAML::Emitter emitter;
    emitter << YAML::Precision(3);
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "image" << YAML::Value << file_name;

    emitter << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq <<
      0 << 0 << 0 << 1 << 0 << 0 << 0 << YAML::EndSeq;

    emitter << YAML::Key << "file_format" << YAML::Value << save_parameters.format;
    emitter << YAML::Key << "as_binary" << YAML::Value << save_parameters.as_binary;

    if (!emitter.good()) {
      std::cout << "[WARNING] [map_io_3d]: YAML writer failed with an error " <<
        emitter.GetLastError() << ". The map metadata may be invalid." << std::endl;
    }

    std::cout << "[INFO] [map_io_3d]: Writing map metadata to " << mapmetadatafile << std::endl;
    std::ofstream(mapmetadatafile) << emitter.c_str();
  }
}

bool saveMapToFile(
  const sensor_msgs::msg::PointCloud2 & map,
  const SaveParameters & save_parameters)
{
  // Local copy of SaveParameters
  SaveParameters save_parameters_loc = save_parameters;

  try {
    // Check map parameters for consistecy
    // Revert to default if needed
    checkSaveParameters(save_parameters_loc);

    tryWriteMapToFile(map, save_parameters_loc);
  } catch (std::exception & e) {
    std::cout << "[ERROR] [map_io_3d]: Failed to write map for reason: " << e.what() << std::endl;
    return false;
  }
  return true;
}

}  // namespace map_3d

}  // namespace nav2_map_server
