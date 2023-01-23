/* Copyright 2019 Rover Robotics
 * Copyright 2010 Brian Gerkey
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "nav2_map_server/map_io.hpp"

#ifndef _WIN32
#include <libgen.h>
#endif
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>

#include "Magick++.h"
#include "nav2_util/geometry_utils.hpp"

#include "yaml-cpp/yaml.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav2_util/occ_grid_values.hpp"

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

  /* Replace all "\" with "/" */
  char * c = path;
  while (*c != '\0') {
    if (*c == '\\') {*c = '/';}
    ++c;
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
using nav2_util::geometry_utils::orientationAroundZAxis;

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
  YAML::Node doc = YAML::LoadFile(yaml_filename);
  LoadParameters load_parameters;

  auto image_file_name = yaml_get_value<std::string>(doc, "image");
  if (image_file_name.empty()) {
    throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
  }
  if (image_file_name[0] != '/') {
    // dirname takes a mutable char *, so we copy into a vector
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    image_file_name = std::string(dirname(fname_copy.data())) + '/' + image_file_name;
  }
  load_parameters.image_file_name = image_file_name;

  load_parameters.resolution = yaml_get_value<double>(doc, "resolution");
  load_parameters.origin = yaml_get_value<std::vector<double>>(doc, "origin");
  if (load_parameters.origin.size() != 3) {
    throw YAML::Exception(
            doc["origin"].Mark(), "value of the 'origin' tag should have 3 elements, not " +
            std::to_string(load_parameters.origin.size()));
  }

  load_parameters.free_thresh = yaml_get_value<double>(doc, "free_thresh");
  load_parameters.occupied_thresh = yaml_get_value<double>(doc, "occupied_thresh");

  auto map_mode_node = doc["mode"];
  if (!map_mode_node.IsDefined()) {
    load_parameters.mode = MapMode::Trinary;
  } else {
    load_parameters.mode = map_mode_from_string(map_mode_node.as<std::string>());
  }

  try {
    load_parameters.negate = yaml_get_value<int>(doc, "negate");
  } catch (YAML::Exception &) {
    load_parameters.negate = yaml_get_value<bool>(doc, "negate");
  }


  auto map_elevation_image_node = doc["elevation_image"];
  if (!map_elevation_image_node.IsDefined()) {
    load_parameters.elevation_image_file_name = "";
  } else {
    auto elevation_image_file_name = yaml_get_value<std::string>(doc, "elevation_image");
    if (elevation_image_file_name.empty()) {
      throw YAML::Exception(doc["elevation_image"].Mark(), "The elevation_image tag was empty.");
    }
    if (elevation_image_file_name[0] != '/') {
      // dirname takes a mutable char *, so we copy into a vector
      std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
      fname_copy.push_back('\0');
      elevation_image_file_name =
        std::string(dirname(fname_copy.data())) + '/' + elevation_image_file_name;
    }
    load_parameters.elevation_image_file_name = elevation_image_file_name;

    load_parameters.min_height = yaml_get_value<double>(doc, "min_height");
    load_parameters.max_height = yaml_get_value<double>(doc, "max_height");
  }

  auto octomap_file_name = doc["octomap_file"];
  if (!octomap_file_name.IsDefined()) {
    load_parameters.octomap_file_name = "";
  } else {
    auto octomap_file_name = yaml_get_value<std::string>(doc, "octomap_file");
    if (octomap_file_name.empty()) {
      throw YAML::Exception(doc["octomap_file"].Mark(), "The octomap tag was empty.");
    }
    if (octomap_file_name[0] != '/') {
      // dirname takes a mutable char *, so we copy into a vector
      std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
      fname_copy.push_back('\0');
      octomap_file_name =
        std::string(dirname(fname_copy.data())) + '/' + octomap_file_name;
    }
    load_parameters.octomap_file_name = octomap_file_name;

    load_parameters.binary = yaml_get_value<bool>(doc, "binary");
  }

  std::cout << "[DEBUG] [map_io]: resolution: " << load_parameters.resolution << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[0]: " << load_parameters.origin[0] << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[1]: " << load_parameters.origin[1] << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[2]: " << load_parameters.origin[2] << std::endl;
  std::cout << "[DEBUG] [map_io]: free_thresh: " << load_parameters.free_thresh << std::endl;
  std::cout << "[DEBUG] [map_io]: occupied_thresh: " << load_parameters.occupied_thresh <<
    std::endl;
  std::cout << "[DEBUG] [map_io]: mode: " << map_mode_to_string(load_parameters.mode) << std::endl;
  std::cout << "[DEBUG] [map_io]: negate: " << load_parameters.negate << std::endl;  //NOLINT

  if (load_parameters.elevation_image_file_name != "") {
    std::cout << "[DEBUG] [map_io]: elevation_file_name: "
              << load_parameters.elevation_image_file_name << std::endl;
    std::cout << "[DEBUG] [map_io]: min_height: " << load_parameters.min_height << std::endl;
    std::cout << "[DEBUG] [map_io]: max_height: " << load_parameters.max_height << std::endl;
  }

  if (load_parameters.octomap_file_name != "") {
    std::cout << "[DEBUG] [map_io]: octomap_file_name: "
              << load_parameters.octomap_file_name << std::endl;
    std::cout << "[DEBUG] [map_io]: binary: " << load_parameters.binary << std::endl;
  }
  return load_parameters;
}

void loadMapFromFile(
  const LoadParameters & load_parameters,
  nav_msgs::msg::OccupancyGrid & map)
{
  Magick::InitializeMagick(nullptr);
  nav_msgs::msg::OccupancyGrid msg;

  std::cout << "[INFO] [map_io]: Loading image_file: " <<
    load_parameters.image_file_name << std::endl;
  Magick::Image img(load_parameters.image_file_name);

  // Copy the image data into the map structure
  msg.info.width = img.size().width();
  msg.info.height = img.size().height();

  msg.info.resolution = load_parameters.resolution;
  msg.info.origin.position.x = load_parameters.origin[0];
  msg.info.origin.position.y = load_parameters.origin[1];
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation = orientationAroundZAxis(load_parameters.origin[2]);

  // Allocate space to hold the data
  msg.data.resize(msg.info.width * msg.info.height);

  // Copy pixel data into the map structure
  for (size_t y = 0; y < msg.info.height; y++) {
    for (size_t x = 0; x < msg.info.width; x++) {
      auto pixel = img.pixelColor(x, y);

      std::vector<Magick::Quantum> channels = {pixel.redQuantum(), pixel.greenQuantum(),
        pixel.blueQuantum()};
      if (load_parameters.mode == MapMode::Trinary && img.matte()) {
        // To preserve existing behavior, average in alpha with color channels in Trinary mode.
        // CAREFUL. alpha is inverted from what you might expect. High = transparent, low = opaque
        channels.push_back(MaxRGB - pixel.alphaQuantum());
      }
      double sum = 0;
      for (auto c : channels) {
        sum += c;
      }
      /// on a scale from 0.0 to 1.0 how bright is the pixel?
      double shade = Magick::ColorGray::scaleQuantumToDouble(sum / channels.size());

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied. Otherwise, it's vice versa.
      /// on a scale from 0.0 to 1.0, how occupied is the map cell (before thresholding)?
      double occ = (load_parameters.negate ? shade : 1.0 - shade);

      int8_t map_cell;
      switch (load_parameters.mode) {
        case MapMode::Trinary:
          if (load_parameters.occupied_thresh < occ) {
            map_cell = nav2_util::OCC_GRID_OCCUPIED;
          } else if (occ < load_parameters.free_thresh) {
            map_cell = nav2_util::OCC_GRID_FREE;
          } else {
            map_cell = nav2_util::OCC_GRID_UNKNOWN;
          }
          break;
        case MapMode::Scale:
          if (pixel.alphaQuantum() != OpaqueOpacity) {
            map_cell = nav2_util::OCC_GRID_UNKNOWN;
          } else if (load_parameters.occupied_thresh < occ) {
            map_cell = nav2_util::OCC_GRID_OCCUPIED;
          } else if (occ < load_parameters.free_thresh) {
            map_cell = nav2_util::OCC_GRID_FREE;
          } else {
            map_cell = std::rint(
              (occ - load_parameters.free_thresh) /
              (load_parameters.occupied_thresh - load_parameters.free_thresh) * 100.0);
          }
          break;
        case MapMode::Raw: {
            double occ_percent = std::round(shade * 255);
            if (nav2_util::OCC_GRID_FREE <= occ_percent &&
              occ_percent <= nav2_util::OCC_GRID_OCCUPIED)
            {
              map_cell = static_cast<int8_t>(occ_percent);
            } else {
              map_cell = nav2_util::OCC_GRID_UNKNOWN;
            }
            break;
          }
        default:
          throw std::runtime_error("Invalid map mode");
      }
      msg.data[msg.info.width * (msg.info.height - y - 1) + x] = map_cell;
    }
  }

  // Since loadMapFromFile() does not belong to any node, publishing in a system time.
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  msg.info.map_load_time = clock.now();
  msg.header.frame_id = "map";
  msg.header.stamp = clock.now();

  std::cout <<
    "[DEBUG] [map_io]: Read map " << load_parameters.image_file_name << ": " << msg.info.width <<
    " X " << msg.info.height << " map @ " << msg.info.resolution << " m/cell" << std::endl;

  map = msg;
}

void loadGridMapFromFile(
  const LoadParameters & load_parameters,
  grid_map::GridMap & grid_map_to_fill)
{
  Magick::InitializeMagick(nullptr);

  std::cout << "[INFO] [map_io]: Loading elevation_image_file: " <<
    load_parameters.elevation_image_file_name << std::endl;
  Magick::Image img(load_parameters.elevation_image_file_name);

  // supposing the iterator gives the same order as the img
  for (grid_map::GridMapIterator grid_iterator(grid_map_to_fill); !grid_iterator.isPastEnd();
    ++grid_iterator)
  {
    // get the value at the iterator
    grid_map::Index index = grid_iterator.getUnwrappedIndex();

    grid_map::Position current_pos;
    grid_map_to_fill.getPosition(*grid_iterator, current_pos);

    auto pixel = img.pixelColor(grid_map_to_fill.getSize()(0) - index[0] - 1, index[1]);

    std::vector<Magick::Quantum> channels = {pixel.redQuantum(), pixel.greenQuantum(),
      pixel.blueQuantum()};
    if (load_parameters.mode == MapMode::Trinary && img.matte()) {
      // To preserve existing behavior, average in alpha with color channels in Trinary mode.
      // CAREFUL. alpha is inverted from what you might expect. High = transparent, low = opaque
      channels.push_back(MaxRGB - pixel.alphaQuantum());
    }
    double sum = 0;
    for (auto c : channels) {
      sum += c;
    }
    /// on a scale from 0.0 to 1.0 how bright is the pixel?
    double shade = Magick::ColorGray::scaleQuantumToDouble(sum / channels.size());

    // If negate is true, we consider blacker pixels free, and whiter
    // pixels occupied. Otherwise, it's vice versa.
    /// on a scale from 0.0 to 1.0, how occupied is the map cell (before thresholding)?
    double occ = (load_parameters.negate ? shade : 1.0 - shade);

    double map_cell;
    // we suppose that the elevation uses the Scale mode
    // ignore the occupied and free threshold
    map_cell =
      occ * (load_parameters.max_height - load_parameters.min_height) + load_parameters.min_height;

    grid_map_to_fill.atPosition("elevation", current_pos) = map_cell;
  }

  std::cout <<
    "[DEBUG] [map_io]: Read map " << load_parameters.elevation_image_file_name << ": " <<
    img.size().width() << " X " << img.size().height() << " map @ " <<
    load_parameters.resolution << " m/cell" << std::endl;
}

void loadOctomapFromFile(
  const LoadParameters & load_parameters,
  std::unique_ptr<octomap::OcTree> & octree)
{
  std::string filename = load_parameters.octomap_file_name;

  if (filename.length() <= 3) {
    throw std::runtime_error("Invalid name for octomap: too short");
  }

  std::string suffix = filename.substr(filename.length() - 3, 3);
  if (suffix == ".bt") {
    if (!octree->readBinary(filename)) {
      throw std::runtime_error("Could not open binary octomap");
    }
  } else if (suffix == ".ot") {
    std::unique_ptr<octomap::AbstractOcTree> tree{octomap::AbstractOcTree::read(filename)};
    if (!tree) {
      throw std::runtime_error("Could not read octomap file");
    }
    octree = std::unique_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(tree.release()));
    if (!octree) {
      throw std::runtime_error("Could not read file");
    }

  } else {
    throw std::runtime_error("Extension not supported");
  }

  std::cout << "[DEBUG] [map_io]: Read Octomap file " << load_parameters.octomap_file_name <<
    " loaded ( " << octree->size() << " nodes)." << std::endl;
}

LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  nav_msgs::msg::OccupancyGrid & map)
{
  if (yaml_file.empty()) {
    std::cerr << "[ERROR] [map_io]: YAML file name is empty, can't load!" << std::endl;
    return MAP_DOES_NOT_EXIST;
  }
  std::cout << "[INFO] [map_io]: Loading yaml file: " << yaml_file << std::endl;
  LoadParameters load_parameters;
  try {
    load_parameters = loadMapYaml(yaml_file);
  } catch (YAML::Exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed processing YAML file " << yaml_file << " at position (" <<
      e.mark.line << ":" << e.mark.column << ") for reason: " << e.what() << std::endl;
    return INVALID_MAP_METADATA;
  } catch (std::exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed to parse map YAML loaded from file " << yaml_file <<
      " for reason: " << e.what() << std::endl;
    return INVALID_MAP_METADATA;
  }

  try {
    loadMapFromFile(load_parameters, map);
  } catch (std::exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed to load image file " << load_parameters.image_file_name <<
      " for reason: " << e.what() << std::endl;
    return INVALID_MAP_DATA;
  }

  return LOAD_MAP_SUCCESS;
}

LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  nav_msgs::msg::OccupancyGrid & map, grid_map_msgs::msg::GridMap & msg_grid_map,
  octomap_msgs::msg::Octomap & msg_octomap)
{
  if (yaml_file.empty()) {
    std::cerr << "[ERROR] [map_io]: YAML file name is empty, can't load!" << std::endl;
    return MAP_DOES_NOT_EXIST;
  }
  std::cout << "[INFO] [map_io]: Loading yaml file: " << yaml_file << std::endl;
  LoadParameters load_parameters;
  try {
    load_parameters = loadMapYaml(yaml_file);
  } catch (YAML::Exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed processing YAML file " << yaml_file << " at position (" <<
      e.mark.line << ":" << e.mark.column << ") for reason: " << e.what() << std::endl;
    return INVALID_MAP_METADATA;
  } catch (std::exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed to parse map YAML loaded from file " << yaml_file <<
      " for reason: " << e.what() << std::endl;
    return INVALID_MAP_METADATA;
  }

  try {
    loadMapFromFile(load_parameters, map);
  } catch (std::exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed to load image file " << load_parameters.image_file_name <<
      " for reason: " << e.what() << std::endl;
    return INVALID_MAP_DATA;
  }

  try {
    grid_map::GridMap grid_map_to_fill({"elevation", "occupancy"});

    geometry_msgs::msg::Quaternion prev_orientation, zero_orientation;
    zero_orientation.x = 0.0;
    zero_orientation.y = 0.0;
    zero_orientation.z = 0.0;
    zero_orientation.w = 1.0;

    prev_orientation = map.info.origin.orientation;
    map.info.origin.orientation = zero_orientation;
    // convert the occupation map to a layer in the grid_map
    grid_map::GridMapRosConverter::fromOccupancyGrid(map, "occupancy", grid_map_to_fill);

    // restore the prev_orient
    map.info.origin.orientation = prev_orientation;

    // it sets the length, resolution etc to the params in the map
    msg_grid_map = *grid_map::GridMapRosConverter::toMessage(grid_map_to_fill);

    if (load_parameters.elevation_image_file_name != "") {
      loadGridMapFromFile(load_parameters, grid_map_to_fill);

    } else {
      // set an elevation layer of 0.0

      grid_map_to_fill.add("elevation", 0.0);
    }

    msg_grid_map = *grid_map::GridMapRosConverter::toMessage(grid_map_to_fill);
  } catch (std::exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed to load elevation image file " <<
      load_parameters.elevation_image_file_name <<
      " for reason: " << e.what() << std::endl;
    return INVALID_MAP_DATA;
  }

  // octomap
  if (load_parameters.octomap_file_name != "") {
    try {
      std::unique_ptr<octomap::OcTree> octree =
        std::make_unique<octomap::OcTree>(load_parameters.resolution);
      loadOctomapFromFile(load_parameters, octree);

      if (!octomap_msgs::fullMapToMsg(*octree, msg_octomap)) {
        throw std::runtime_error("Error serializing Octomap");
      }
    } catch (std::exception & e) {
      std::cerr <<
        "[ERROR] [map_io]: Failed to load octomap image file " <<
        load_parameters.octomap_file_name <<
        " for reason: " << e.what() << std::endl;
      return INVALID_MAP_DATA;
    }
  }

  return LOAD_MAP_SUCCESS;
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
  // Magick must me initialized before any activity with images
  Magick::InitializeMagick(nullptr);

  // Checking map file name
  if (save_parameters.map_file_name == "") {
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    save_parameters.map_file_name = "map_" +
      std::to_string(static_cast<int>(clock.now().seconds()));
    std::cout << "[WARN] [map_io]: Map file unspecified. Map will be saved to " <<
      save_parameters.map_file_name << " file" << std::endl;
  }

  // Checking thresholds
  if (save_parameters.occupied_thresh == 0.0) {
    save_parameters.occupied_thresh = 0.65;
    std::cout << "[WARN] [map_io]: Occupied threshold unspecified. Setting it to default value: " <<
      save_parameters.occupied_thresh << std::endl;
  }
  if (save_parameters.free_thresh == 0.0) {
    save_parameters.free_thresh = 0.25;
    std::cout << "[WARN] [map_io]: Free threshold unspecified. Setting it to default value: " <<
      save_parameters.free_thresh << std::endl;
  }
  if (1.0 < save_parameters.occupied_thresh) {
    std::cerr << "[ERROR] [map_io]: Threshold_occupied must be 1.0 or less" << std::endl;
    throw std::runtime_error("Incorrect thresholds");
  }
  if (save_parameters.free_thresh < 0.0) {
    std::cerr << "[ERROR] [map_io]: Free threshold must be 0.0 or greater" << std::endl;
    throw std::runtime_error("Incorrect thresholds");
  }
  if (save_parameters.occupied_thresh <= save_parameters.free_thresh) {
    std::cerr << "[ERROR] [map_io]: Threshold_free must be smaller than threshold_occupied" <<
      std::endl;
    throw std::runtime_error("Incorrect thresholds");
  }

  // Checking image format
  if (save_parameters.image_format == "") {
    save_parameters.image_format = save_parameters.mode == MapMode::Scale ? "png" : "pgm";
    std::cout << "[WARN] [map_io]: Image format unspecified. Setting it to: " <<
      save_parameters.image_format << std::endl;
  }

  std::transform(
    save_parameters.image_format.begin(),
    save_parameters.image_format.end(),
    save_parameters.image_format.begin(),
    [](unsigned char c) {return std::tolower(c);});

  const std::vector<std::string> BLESSED_FORMATS{"bmp", "pgm", "png"};
  if (
    std::find(BLESSED_FORMATS.begin(), BLESSED_FORMATS.end(), save_parameters.image_format) ==
    BLESSED_FORMATS.end())
  {
    std::stringstream ss;
    bool first = true;
    for (auto & format_name : BLESSED_FORMATS) {
      if (!first) {
        ss << ", ";
      }
      ss << "'" << format_name << "'";
      first = false;
    }
    std::cout <<
      "[WARN] [map_io]: Requested image format '" << save_parameters.image_format <<
      "' is not one of the recommended formats: " << ss.str() << std::endl;
  }
  const std::string FALLBACK_FORMAT = "png";

  try {
    Magick::CoderInfo info(save_parameters.image_format);
    if (!info.isWritable()) {
      std::cout <<
        "[WARN] [map_io]: Format '" << save_parameters.image_format <<
        "' is not writable. Using '" << FALLBACK_FORMAT << "' instead" << std::endl;
      save_parameters.image_format = FALLBACK_FORMAT;
    }
  } catch (Magick::ErrorOption & e) {
    std::cout <<
      "[WARN] [map_io]: Format '" << save_parameters.image_format << "' is not usable. Using '" <<
      FALLBACK_FORMAT << "' instead:" << std::endl << e.what() << std::endl;
    save_parameters.image_format = FALLBACK_FORMAT;
  }

  // Checking map mode
  if (
    save_parameters.mode == MapMode::Scale &&
    (save_parameters.image_format == "pgm" ||
    save_parameters.image_format == "jpg" ||
    save_parameters.image_format == "jpeg"))
  {
    std::cout <<
      "[WARN] [map_io]: Map mode 'scale' requires transparency, but format '" <<
      save_parameters.image_format <<
      "' does not support it. Consider switching image format to 'png'." << std::endl;
  }

  // Check heights
  if (save_parameters.min_height > save_parameters.max_height) {
    std::cout <<
      "[WARN] [map_io]: Grid Map min_height:" <<
      save_parameters.min_height <<
      "is greater than max_height:" << save_parameters.max_height << std::endl;
  }
}

/**
 * @brief Tries to write map data into a file
 * @param map Occupancy grid data
 * @param save_parameters Map saving parameters
 * @throw std::expection in case of problem
 */
void tryWriteMapToFile(
  const nav_msgs::msg::OccupancyGrid & map,
  const SaveParameters & save_parameters)
{
  std::cout <<
    "[INFO] [map_io]: Received a " << map.info.width << " X " << map.info.height << " map @ " <<
    map.info.resolution << " m/pix" << std::endl;

  std::string mapdatafile = save_parameters.map_file_name + "." + save_parameters.image_format;
  {
    // should never see this color, so the initialization value is just for debugging
    Magick::Image image({map.info.width, map.info.height}, "red");

    // In scale mode, we need the alpha (matte) channel. Else, we don't.
    // NOTE: GraphicsMagick seems to have trouble loading the alpha channel when saved with
    // Magick::GreyscaleMatte, so we use TrueColorMatte instead.
    image.type(
      save_parameters.mode == MapMode::Scale ?
      Magick::TrueColorMatteType : Magick::GrayscaleType);

    // Since we only need to support 100 different pixel levels, 8 bits is fine
    image.depth(8);

    int free_thresh_int = std::rint(save_parameters.free_thresh * 100.0);
    int occupied_thresh_int = std::rint(save_parameters.occupied_thresh * 100.0);

    for (size_t y = 0; y < map.info.height; y++) {
      for (size_t x = 0; x < map.info.width; x++) {
        int8_t map_cell = map.data[map.info.width * (map.info.height - y - 1) + x];

        Magick::Color pixel;

        switch (save_parameters.mode) {
          case MapMode::Trinary:
            if (map_cell < 0 || 100 < map_cell) {
              pixel = Magick::ColorGray(205 / 255.0);
            } else if (map_cell <= free_thresh_int) {
              pixel = Magick::ColorGray(254 / 255.0);
            } else if (occupied_thresh_int <= map_cell) {
              pixel = Magick::ColorGray(0 / 255.0);
            } else {
              pixel = Magick::ColorGray(205 / 255.0);
            }
            break;
          case MapMode::Scale:
            if (map_cell < 0 || 100 < map_cell) {
              pixel = Magick::ColorGray{0.5};
              pixel.alphaQuantum(TransparentOpacity);
            } else {
              pixel = Magick::ColorGray{(100.0 - map_cell) / 100.0};
            }
            break;
          case MapMode::Raw:
            Magick::Quantum q;
            if (map_cell < 0 || 100 < map_cell) {
              q = MaxRGB;
            } else {
              q = map_cell / 255.0 * MaxRGB;
            }
            pixel = Magick::Color(q, q, q);
            break;
          default:
            std::cerr << "[ERROR] [map_io]: Map mode should be Trinary, Scale or Raw" << std::endl;
            throw std::runtime_error("Invalid map mode");
        }
        image.pixelColor(x, y, pixel);
      }
    }

    std::cout << "[INFO] [map_io]: Writing map occupancy data to " << mapdatafile << std::endl;
    image.write(mapdatafile);
  }
}

void WriteMetadataOcc(
  const nav_msgs::msg::OccupancyGrid & map,
  const SaveParameters & save_parameters,
  std::string mapdatafile,
  std::string mapmetadatafile)
{
  std::ofstream yaml(mapmetadatafile);

  geometry_msgs::msg::Quaternion orientation = map.info.origin.orientation;
  tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  const int file_name_index = mapdatafile.find_last_of("/\\");
  std::string image_name = mapdatafile.substr(file_name_index + 1);

  YAML::Emitter e;
  e << YAML::Precision(3);
  e << YAML::BeginMap;
  e << YAML::Key << "image" << YAML::Value << image_name;
  e << YAML::Key << "mode" << YAML::Value << map_mode_to_string(save_parameters.mode);
  e << YAML::Key << "resolution" << YAML::Value << map.info.resolution;
  e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq << map.info.origin.position.x <<
    map.info.origin.position.y << yaw << YAML::EndSeq;
  e << YAML::Key << "negate" << YAML::Value << 0;
  e << YAML::Key << "occupied_thresh" << YAML::Value << save_parameters.occupied_thresh;
  e << YAML::Key << "free_thresh" << YAML::Value << save_parameters.free_thresh;

  if (!e.good()) {
    std::cout <<
      "[WARN] [map_io]: YAML writer failed with an error " << e.GetLastError() <<
      ". The map metadata may be invalid." << std::endl;
  }

  std::cout << "[INFO] [map_io]: Writing map metadata to " << mapmetadatafile << std::endl;
  std::ofstream(mapmetadatafile) << e.c_str();
}

void WriteMetadataToFile(
  const nav_msgs::msg::OccupancyGrid & map,
  const SaveParameters & save_parameters)
{
  std::string mapdatafile = save_parameters.map_file_name + "." + save_parameters.image_format;
  std::string mapmetadatafile = save_parameters.map_file_name + ".yaml";
  {
    WriteMetadataOcc(map, save_parameters, mapdatafile, mapmetadatafile);
  }
  std::cout << "[INFO] [map_io]: Map saved" << std::endl;
}

void WriteMetadataToFile(
  const grid_map_msgs::msg::GridMap & map,
  const SaveParameters & save_parameters)
{
  std::string mapdatafile = save_parameters.map_file_name + "_ele." + save_parameters.image_format;
  std::string mapmetadatafile = save_parameters.map_file_name + ".yaml";

  // occupancy

  nav_msgs::msg::OccupancyGrid occ_map;
  grid_map::GridMap my_grid_map;
  grid_map::GridMapRosConverter::fromMessage(map, my_grid_map);
  grid_map::GridMapRosConverter::toOccupancyGrid(my_grid_map, "occupancy", 0.0, 254.0, occ_map);

  {
    WriteMetadataOcc(occ_map, save_parameters, mapdatafile, mapmetadatafile);
  }

  // occupancy data added

  // elevation
  YAML::Emitter e;
  e << YAML::Precision(3);
  e << YAML::Newline;
  e << YAML::Comment(" elevation layer");
  e << YAML::BeginMap;
  e << YAML::Key << "elevation_image" << YAML::Value << mapdatafile;
  e << YAML::Key << "min_height" << YAML::Value << save_parameters.min_height;
  e << YAML::Key << "max_height" << YAML::Value << save_parameters.max_height;

  if (!e.good()) {
    std::cout <<
      "[WARN] [map_io]: YAML writer failed with an error " << e.GetLastError() <<
      ". The map metadata may be invalid." << std::endl;
  }

  std::cout << "[INFO] [map_io]: Writing map metadata to " << mapmetadatafile << std::endl;
  std::ofstream(mapmetadatafile, std::ios_base::openmode::_S_app) << e.c_str();
  std::cout << "[INFO] [map_io]: Map saved" << std::endl;
}

void WriteMetadataToFile(
  const octomap_msgs::msg::Octomap & map,
  const SaveParameters & save_parameters)
{
  std::string mapdatafile = save_parameters.map_file_name + "_octo.ot";
  std::string mapmetadatafile = save_parameters.map_file_name + ".yaml";

  // elevation
  YAML::Emitter e;
  e << YAML::Precision(3);
  e << YAML::Newline;
  e << YAML::Comment(" octomap");
  e << YAML::BeginMap;
  e << YAML::Key << "octomap_file" << YAML::Value << mapdatafile;
  e << YAML::TrueFalseBool << "binary" << YAML::Value << map.binary;
  if (!e.good()) {
    std::cout <<
      "[WARN] [map_io]: YAML writer failed with an error " << e.GetLastError() <<
      ". The map metadata may be invalid." << std::endl;
  }

  std::cout << "[INFO] [map_io]: Writing octomap metadata to " << mapmetadatafile << std::endl;
  std::ofstream(mapmetadatafile, std::ios_base::openmode::_S_app) << e.c_str();
  std::cout << "[INFO] [map_io]: Octomap saved" << std::endl;
}

bool saveMapToFile(
  const nav_msgs::msg::OccupancyGrid & map,
  const SaveParameters & save_parameters)
{
  // Local copy of SaveParameters that might be modified by checkSaveParameters()
  SaveParameters save_parameters_loc = save_parameters;

  try {
    // Checking map parameters for consistency
    checkSaveParameters(save_parameters_loc);

    tryWriteMapToFile(map, save_parameters_loc);
    WriteMetadataToFile(map, save_parameters_loc);
  } catch (std::exception & e) {
    std::cout << "[ERROR] [map_io]: Failed to write map for reason: " << e.what() << std::endl;
    return false;
  }
  return true;
}

bool saveMapToFile(
  const grid_map_msgs::msg::GridMap & map,
  const SaveParameters & save_parameters)
{
  // Local copy of SaveParameters that might be modified by checkSaveParameters()
  SaveParameters save_parameters_loc = save_parameters;
  double min_h = save_parameters_loc.min_height;
  double max_h = save_parameters_loc.max_height;
  checkSaveParameters(save_parameters_loc);

  grid_map::GridMap gridmap;
  grid_map::GridMapRosConverter::fromMessage(map, gridmap, {"occupancy", "elevation"});

  nav_msgs::msg::OccupancyGrid occ_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(gridmap, "occupancy", 0.0, 254.0, occ_grid);
  nav_msgs::msg::OccupancyGrid ele_grid;

  if (save_parameters_loc.max_height == 0.0 && save_parameters_loc.min_height == 0.0) {
    // infer max and min elevation boundaries from the data
    // change the params to print them in the yaml
    min_h = gridmap["elevation"].minCoeff();
    max_h = gridmap["elevation"].maxCoeff();
    grid_map::GridMapRosConverter::toOccupancyGrid(gridmap, "elevation", min_h, max_h, ele_grid);
  } else {
    grid_map::GridMapRosConverter::toOccupancyGrid(
      gridmap, "elevation",
      save_parameters_loc.min_height,
      save_parameters_loc.max_height, ele_grid);
  }

  try {
    // Checking map parameters for consistency
    checkSaveParameters(save_parameters_loc);

    tryWriteMapToFile(occ_grid, save_parameters_loc);
  } catch (std::exception & e) {
    std::cout << "[ERROR] [map_io]: Failed to write map for reason: " << e.what() << std::endl;
    return false;
  }

  try {
    // Checking map parameters for consistency
    save_parameters_loc = save_parameters;
    checkSaveParameters(save_parameters_loc);
    save_parameters_loc.map_file_name = save_parameters_loc.map_file_name + "_ele";
    // elevation must be read in scale mode
    save_parameters_loc.mode = nav2_map_server::MapMode::Scale;
    save_parameters_loc.min_height = min_h;
    save_parameters_loc.max_height = max_h;
    tryWriteMapToFile(ele_grid, save_parameters_loc);
  } catch (std::exception & e) {
    std::cout << "[ERROR] [map_io]: Failed to write map for reason: " << e.what() << std::endl;
    return false;
  }

  save_parameters_loc = save_parameters;
  checkSaveParameters(save_parameters_loc);

  save_parameters_loc.min_height = min_h;
  save_parameters_loc.max_height = max_h;
  WriteMetadataToFile(map, save_parameters_loc);

  return true;
}

bool saveOctomapToFile(
  const octomap_msgs::msg::Octomap & octomap_msg,
  const SaveParameters & save_parameters)
{
  SaveParameters save_parameters_loc = save_parameters;
  checkSaveParameters(save_parameters_loc);
  std::string map_name = save_parameters_loc.map_file_name + "_octo.ot";

  // saving the file as in https://github.com/OctoMap/octomap_mapping/blob/ros2/octomap_server/src/octomap_saver.cpp#L84
  std::unique_ptr<octomap::AbstractOcTree> tree{octomap_msgs::msgToMap(octomap_msg)};
  std::unique_ptr<octomap::AbstractOccupancyOcTree> octree;
  if (tree) {
    octree =
      std::unique_ptr<octomap::AbstractOccupancyOcTree>(
      dynamic_cast<octomap::AbstractOccupancyOcTree *>(tree.
      release()));
  } else {
    std::cerr << "Error creating octree from received message" << std::endl;
    return false;
  }

  if (octree) {
    std::cout <<
      "Map received (" << octree->size() << " nodes, " << octree->getResolution()
              << " m res), saving to" << map_name.c_str() << std::endl;

    std::string suffix = map_name.substr(map_name.length() - 3, 3);
    if (suffix == ".bt") {  // write to binary file:
      if (!octree->writeBinary(map_name)) {
        std::cerr << "Error writing to file " << map_name.c_str() << std::endl;
        return false;
      }
    } else if (suffix == ".ot") {  // write to full .ot file:
      if (!octree->write(map_name)) {
        std::cerr << "Error writing to file " << map_name.c_str() << std::endl;
        return false;
      }
    } else {
      std::cerr << "Unknown file extension, must be either .bt or .ot" << std::endl;
      return false;
    }
  } else {
    std::cerr << "Error reading OcTree from stream" << std::endl;
    return false;
  }

  WriteMetadataToFile(octomap_msg, save_parameters_loc);
  return true;
}

}  // namespace nav2_map_server
