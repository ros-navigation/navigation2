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

#include "nav2_map_server/map_3d/map_io_3d.hpp"

#ifndef _WIN32
#include <libgen.h>
#endif
#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include "nav2_util/geometry_utils.hpp"

#include "yaml-cpp/yaml.h"
#include "nav2_map_server/map_3d/pcl_helper.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/io/pcd_io.h"
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
namespace map_3d
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
  std::cout << "loading the yaml file";

  YAML::Node doc = YAML::LoadFile(yaml_filename);
  LoadParameters load_parameters;
  load_parameters.origin.resize();

  std::string pcd_file_name(YamlGetValue<std::string>(doc, "image"));
  if (!ends_with(pcd_file_name, ".pcd")) {
    throw YAML::Exception(
            doc["image"].Mark(),
            "The pcd file is invalid please pass a file name with extension .pcd");
  }
  std::cout << "file written";

  if (pcd_file_name[0] != '/') {
    // dirname takes a mutable char *, so we copy into a vector
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    load_parameters.pcd_file_name =
      std::string(dirname(fname_copy.data())) + '/' + pcd_file_name;
  } else {
    load_parameters.pcd_file_name = pcd_file_name;
  }
  std::cout << "setting the origin";

  // Get view point as center and orientation
  // view point will be loaded from pcd file while reading
  // If not provided by YAML, if it provided by both YAML and PCD then
  // a warning will be issued if they are not equal and PCD  will take precedence
  try {
    // Load origin form yaml_file
    std::vector<float> pcd_origin = YamlGetValue<std::vector<float>>(doc, "origin");

    if (pcd_origin.size() == 7) {
      for (int i = 0; i < 7; ++i) {
        if (i < 3) {
          load_parameters.origin.center[i] = pcd_origin[i];
        } else {
          load_parameters.origin.orientation[i - 3] = pcd_origin[i];
        }
      }
    }
  } catch (YAML::Exception & e) {
    std::cout << "[WARNING] [map_3d]: Couldn't load view_point from yaml file: "
      "If not provided it we will try to load from pcd file" << std::endl;
  }

  return load_parameters;
}

void loadMapFromFile(
  const LoadParameters & load_parameters,
  sensor_msgs::msg::PointCloud2 & map_msg,
  geometry_msgs::msg::Pose & origin_msg)
{
  sensor_msgs::msg::PointCloud2 map;
  geometry_msgs::msg::Pose map_origin;

  std::cout << "[INFO] [map_3d]: Loading pcd_file: " <<
    load_parameters.pcd_file_name << std::endl;

  pcl::PCLPointCloud2::Ptr cloud = std::make_shared<pcl::PCLPointCloud2>();

  //  create a pcd reader for PointCloud2 data
  pcl::PCDReader reader;

  Eigen::Vector4f center;
  Eigen::Quaternionf orientation;
  int pcd_version = 0;

  std::cout << "[debug] [map_3d]" << "feeding pcd filename to reader" << std::endl;

  if (reader.read(
      load_parameters.pcd_file_name, *cloud,
      center, orientation, pcd_version) == -1)              //* load the file
  {
    std::string error_msg{"Couldn't read "};
    error_msg += load_parameters.pcd_file_name + "\n";
    PCL_ERROR(error_msg.c_str());
  }
  std::cout << "[debug] [map_3d]" << "pcd file is loaded" << std::endl;

  if (load_parameters.origin.center.size() == 3 &&
    load_parameters.origin.orientation.size() == 4)
  {
    // Update translation of transformation
    origin_msg.position.x = load_parameters.origin.center[0];
    origin_msg.position.y = load_parameters.origin.center[1];
    origin_msg.position.z = load_parameters.origin.center[2];

    // Update rotation of transformation
    origin_msg.orientation.w = load_parameters.origin.orientation[0];
    origin_msg.orientation.x = load_parameters.origin.orientation[1];
    origin_msg.orientation.y = load_parameters.origin.orientation[2];
    origin_msg.orientation.z = load_parameters.origin.orientation[3];
  } else {
    std::cout << "[WARNING] [map_3d]: View Point(center and orientation) not provided by "
      "YAML now will be using view_point defined by pcd reader" << std::endl;

    // Update translation of transformation
    origin_msg.position.x = center[0];
    origin_msg.position.y = center[1];
    origin_msg.position.z = center[2];

    // Update rotation of transformation
    origin_msg.orientation.w = orientation.w();
    origin_msg.orientation.x = orientation.x();
    origin_msg.orientation.y = orientation.y();
    origin_msg.orientation.z = orientation.z();
  }
  std::cout << "[debug] [map_3d]" << "converting pcd to message" << std::endl;

  //  update message data
  pclToMsg(map, cloud);

  std::cout << "[debug] [map_3d]" << "message conversion is done" << std::endl;

  map_msg = map;
}

LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  sensor_msgs::msg::PointCloud2 & map_msg,
  geometry_msgs::msg::Pose & origin_msg)
{
  if (yaml_file.empty()) {
    std::cerr << "[ERROR] [map_3d]: YAML file name is empty, can't load!" << std::endl;
    return MAP_DOES_NOT_EXIST;
  }

  std::cout << "[INFO] [map_3d]: Loading yaml file: " << yaml_file << std::endl;
  LoadParameters load_parameters;
  std::cout << "load parameters created";

  load_parameters.origin.resize();
  std::cout << "resize done";

  try {
    std::cout << "initiating load_parameters";

    load_parameters = loadMapYaml(yaml_file);
  } catch (YAML::Exception & e) {
    std::cerr << "[ERROR] [map_3d]: Failed processing YAML file " << yaml_file <<
      " at position (" << e.mark.line << ":" << e.mark.column << ") for reason: " <<
      e.what() << std::endl;

    return INVALID_MAP_METADATA;
  }
//  catch (std::exception & e) {
//    std::cerr << "[ERROR] [map_3d]: Failed to parse map YAML loaded from file " << yaml_file <<
//      " for reason: " << e.what() << std::endl;
//
//    return INVALID_MAP_METADATA;
//  }

  std::cout << "load parameters  are done to be completed";

  try {
    loadMapFromFile(load_parameters, map_msg, origin_msg);
  } catch (std::exception & e) {
    std::cerr <<
      "[ERROR] [map_3d]: Failed to load image file " << load_parameters.pcd_file_name <<
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
    std::cout << "[WARN] [map_3d]: Map file unspecified. Map will be saved to " <<
      save_parameters.map_file_name << " file" << std::endl;
  }

  if (save_parameters.as_binary) {
    std::cout << "[WARN] [map_3d]: Map will be saved in binary form to " <<
      save_parameters.map_file_name << " file" << std::endl;
  }

  if (save_parameters.format.empty()) {
    save_parameters.format = "pcd";
    std::cout << "[WARN] [map_3d]: No map format is "
      "specifies we will be using pcd format" << std::endl;
  }

  if (save_parameters.format != "ply" || save_parameters.format != "pcd") {
    save_parameters.format = "pcd";
    std::cout << "[WARN] [map_3d]: " << save_parameters.format <<
      " support is not implemented, Falling back to pcd file format" << std::endl;
  }
  if (save_parameters.format == "ply") {
    // TODO(Shivam Pandey): add ply support
    save_parameters.format = "pcd";
    std::cout << "[WARN] [map_3d]: ply support is not implemented, "
      "Falling back to pcd file format" << std::endl;
  }

  if (save_parameters.origin.center.size() != 3 && save_parameters.origin.orientation.size() != 4) {
    save_parameters.origin.center = {0, 0, 0};
    save_parameters.origin.orientation = {1, 0, 0, 0};
    std::cout << "[WARN] [map_3d]: "
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
  } else if (save_parameters.format == "ply") {
    file_name += ".ply";
  }

  std::shared_ptr<pcl::PCLPointCloud2> cloud_2 = std::make_shared<pcl::PCLPointCloud2>();
  msgToPcl(cloud_2, map);

  pcl::PCDWriter writer;

  // Initialize origin
  Eigen::Vector4f center = Eigen::Vector4f::Zero();
  center[0] = save_parameters.origin.center[0];
  center[1] = save_parameters.origin.center[1];
  center[2] = save_parameters.origin.center[2];

  // Initialize orientation
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
  orientation.w() = save_parameters.origin.orientation[0];
  orientation.x() = save_parameters.origin.orientation[1];
  orientation.y() = save_parameters.origin.orientation[2];
  orientation.z() = save_parameters.origin.orientation[3];

  if (writer.write(
      file_name, cloud_2, center,
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
      save_parameters.origin.center[0] << save_parameters.origin.center[1] <<
      save_parameters.origin.center[2] << save_parameters.origin.orientation[0] <<
      save_parameters.origin.orientation[1] << save_parameters.origin.orientation[2] <<
      save_parameters.origin.orientation[3] << YAML::EndSeq;

    emitter << YAML::Key << "file_format" << YAML::Value << save_parameters.format;
    emitter << YAML::Key << "as_binary" << YAML::Value << save_parameters.as_binary;

    if (!emitter.good()) {
      std::cout << "[WARN] [map_3d]: YAML writer failed with an error " <<
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

}  // namespace map_3d
}  // namespace nav2_map_server
