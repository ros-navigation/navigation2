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

#include "nav2_map_server_3d/map_io_3d.hpp"

#ifndef _WIN32
#include <libgen.h>
#endif
#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include "nav2_util/geometry_utils.hpp"

#include "yaml-cpp/yaml.h"
#include "nav2_map_server_3d/pcl_helper.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/io/pcd_io.h"
#include "boost/filesystem.hpp"
#include "Eigen/Core"

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
namespace nav2_map_server_3d
{

using nav2_util::geometry_utils::orientationAroundZAxis;

// === Map input part ===

/// Get the given subnode value.
/// The only reason this function exists is to wrap the exceptions in slightly nicer error messages,
/// including the name of the failed key
/// @throw YAML::Exception
template<typename T>
T YamlGetValue(const YAML::Node & node, const std::string & key)
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
  YAML::Node doc = YAML::LoadFile(yaml_filename);
  LoadParameters load_parameters;

  boost::filesystem::path pcd_file_name(YamlGetValue<std::string>(doc, "image"));
  if (pcd_file_name.extension() != ".pcd") {
    throw YAML::Exception(
            doc["image"].Mark(),
            "The pcd file is invalid please pass a file name with extension .pcd");
  }

  if (pcd_file_name.string()[0] != '/') {
    // dirname takes a mutable char *, so we copy into a vector
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    load_parameters.pcd_file_name =
      std::string(dirname(fname_copy.data())) + '/' + pcd_file_name.string();
  } else {
    load_parameters.pcd_file_name = pcd_file_name.string();
  }

  // Get view point as center and orientation
  // view point will be loaded from pcd file while reading
  // If not provided by YAML, if it provided by both YAML and PCD then
  // a warning will be issued if they are not equal and PCD  will take precedence
  try {
    std::vector<float> pcd_origin = YamlGetValue<std::vector<float>>(doc, "origin");
    std::vector<float> pcd_orientation = YamlGetValue<std::vector<float>>(doc, "orientation");

    if (pcd_origin.size() == 3 && pcd_orientation.size() == 4) {
      load_parameters.origin = pcd_origin;
      load_parameters.orientation = pcd_orientation;
    }
  } catch (YAML::Exception & e) {
    std::cout << "[WARNING] [map_io_3d]: Couldn't load view_point from yaml file: "
      "If not provided it we will try to load from pcd file" << std::endl;
  }

  return load_parameters;
}

void loadMapFromFile(
  const LoadParameters & load_parameters,
  nav2_msgs::msg::PCD2 & map_msg)
{
  nav2_msgs::msg::PCD2 msg;

  std::cout << "[INFO] [map_io_3d]: Loading pcd_file: " <<
    load_parameters.pcd_file_name << std::endl;

  pcl::PCLPointCloud2::Ptr cloud = std::make_shared<pcl::PCLPointCloud2>();

  //  create a pcd reader for PointCloud2 data
  pcl::PCDReader reader;

  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  int pcd_version = 0;

  if (reader.read(
      load_parameters.pcd_file_name, *cloud,
      origin, orientation, pcd_version) == -1)              //* load the file
  {
    std::string error_msg{"Couldn't read "};
    error_msg += load_parameters.pcd_file_name + "\n";
    PCL_ERROR(error_msg.c_str());
  }

  if (load_parameters.origin.size() == 3 && load_parameters.orientation.size() == 4) {
    // Update translation of transformation
    msg.origin.x = load_parameters.origin[0];
    msg.origin.y = load_parameters.origin[1];
    msg.origin.z = load_parameters.origin[2];

    // Update rotation of transformation
    msg.orientation.w = load_parameters.orientation[0];
    msg.orientation.x = load_parameters.orientation[1];
    msg.orientation.y = load_parameters.orientation[2];
    msg.orientation.z = load_parameters.orientation[3];
  } else {
    std::cout << "[WARNING] [map_io_3d]: View Point(centre and orientation) not provided by "
      "YAML now will be using view_point defined by pcd reader" << std::endl;

    // Update translation of transformation
    msg.origin.x = origin[0];
    msg.origin.y = origin[1];
    msg.origin.z = origin[2];

    // Update rotation of transformation
    msg.orientation.w = orientation.w();
    msg.orientation.x = orientation.x();
    msg.orientation.y = orientation.y();
    msg.orientation.z = orientation.z();
  }

  //  update message data
  pclToMsg(msg.map, cloud);

  map_msg = msg;
}

LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  nav2_msgs::msg::PCD2 & map_msg)
{
  if (yaml_file.empty()) {
    std::cerr << "[ERROR] [map_io_3d]: YAML fiel name is empty, can't load!" << std::endl;
    return MAP_DOES_NOT_EXIST;
  }

  std::cout << "[INFO] [map_io_3d]: Loading yaml file: " << yaml_file << std::endl;
  LoadParameters load_parameters;

  try {
    load_parameters = loadMapYaml(yaml_file);
  } catch (YAML::Exception & e) {
    std::cerr << "[ERROR] [map_io_3d]: Failed processing YAML file " << yaml_file <<
      " at position (" << e.mark.line << ":" << e.mark.column << ") for reason: " <<
      e.what() << std::endl;

    return INVALID_MAP_METADATA;
  } catch (std::exception & e) {
    std::cerr << "[ERROR] [map_io_3d]: Failed to parse map YAML loaded from file " << yaml_file <<
      " for reason: " << e.what() << std::endl;

    return INVALID_MAP_METADATA;
  }

  try {
    loadMapFromFile(load_parameters, map_msg);
  } catch (std::exception & e) {
    std::cerr <<
      "[ERROR] [map_io_3d]: Failed to load image file " << load_parameters.pcd_file_name <<
      " for reason: " << e.what() << std::endl;

    return INVALID_MAP_DATA;
  }

  return LOAD_MAP_SUCCESS;
}

void CheckSaveParameters(SaveParameters & save_parameters)
{
  if (save_parameters.map_file_name.empty()) {
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    save_parameters.map_file_name =
      "map_" + std::to_string(static_cast<int>(clock.now().seconds()));
    std::cout << "[WARN] [map_io_3d]: Map file unspecified. Map will be saved to " <<
      save_parameters.map_file_name << " file" << std::endl;
  }

  if (save_parameters.as_binary) {
    std::cout << "[WARN] [map_io_3d]: Map will be saved in binary form to " <<
      save_parameters.map_file_name << " file" << std::endl;
  }

  if (save_parameters.format.empty()) {
    save_parameters.format = "pcd";
    std::cout << "[WARN] [map_io_3d]: No map format is "
      "specifies we will be using pcd format" << std::endl;
  }

  if (save_parameters.format != "ply" || save_parameters.format != "pcd") {
    save_parameters.format = "pcd";
    std::cout << "[WARN] [map_io_3d]: " << save_parameters.format <<
      " support is not implemented, Falling back to pcd file format" << std::endl;
  }
  if (save_parameters.format == "ply") {
    // TODO(Shivam Pandey): add ply support
    save_parameters.format = "pcd";
    std::cout << "[WARN] [map_io_3d]: ply support is not implemented, "
      "Falling back to pcd file format" << std::endl;
  }

  if (save_parameters.origin.size() != 3 && save_parameters.orientation.size() != 4) {
    save_parameters.origin = {0, 0, 0};
    save_parameters.orientation = {1, 0, 0, 0};
    std::cout << "[WARN] [map_io_3d]: "
      "origin and orientation provided must have a length of 3 and 4 respectively, "
      "Falling back to identity transform[0, 0, 0] ,[1, 0, 0, 0]" << std::endl;
  }
}

void TryWriteMapToFile(
  const sensor_msgs::msg::PointCloud2 & map,
  const SaveParameters & save_parameters)
{
  std::string file_name(save_parameters.map_file_name);

  if (save_parameters.format == "pcd") {
    file_name += ".pcd";
  } else {
    file_name += ".ply";
  }

  std::shared_ptr<pcl::PCLPointCloud2> cloud_2 = std::make_shared<pcl::PCLPointCloud2>();
  msgToPcl(cloud_2, map);

  pcl::PCDWriter writer;

  // Initialize origin
  Eigen::Vector4f origin = Eigen::Vector4f::Zero();
  origin[0] = save_parameters.origin[0];
  origin[1] = save_parameters.origin[1];
  origin[2] = save_parameters.origin[2];

  // Initialize orientation
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
  orientation.w() = save_parameters.orientation[0];
  orientation.x() = save_parameters.orientation[1];
  orientation.y() = save_parameters.orientation[2];
  orientation.z() = save_parameters.orientation[3];

  if (writer.write(
      file_name, cloud_2, origin,
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
      save_parameters.origin[0] << save_parameters.origin[1] <<
      save_parameters.origin[2] << YAML::EndSeq;
    emitter << YAML::Key << "orientation" << YAML::Flow << YAML::BeginSeq <<
      save_parameters.orientation[0] << save_parameters.orientation[1] <<
      save_parameters.orientation[2] << save_parameters.orientation[3] << YAML::EndSeq;

    emitter << YAML::Key << "as_binary" << YAML::Value << save_parameters.as_binary;
    emitter << YAML::Key << "file_format" << YAML::Value << save_parameters.format;

    if (!emitter.good()) {
      std::cout << "[WARN] [map_io_3d]: YAML writer failed with an error " <<
        emitter.GetLastError() << ". The map metadata may be invalid." << std::endl;
    }

    std::cout << "[INFO] [map_io]: Writing map metadata to " << mapmetadatafile << std::endl;
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
    CheckSaveParameters(save_parameters_loc);

    TryWriteMapToFile(map, save_parameters_loc);
  } catch (std::exception & e) {
    std::cout << "[ERROR] [map_io]: Failed to write map for reason: " << e.what() << std::endl;
    return false;
  }
  return true;
}

}  // namespace nav2_map_server_3d
}  // namespace nav2_map_server
