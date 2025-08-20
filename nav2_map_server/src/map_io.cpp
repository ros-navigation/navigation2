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

#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <cstdlib>

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

std::string get_home_dir()
{
  if (const char * home_dir = std::getenv("HOME")) {
    return std::string{home_dir};
  }
  return std::string{};
}

std::string expand_user_home_dir_if_needed(
  std::string yaml_filename,
  std::string home_variable_value)
{
  if (yaml_filename.size() < 2 || !(yaml_filename[0] == '~' && yaml_filename[1] == '/')) {
    return yaml_filename;
  }
  if (home_variable_value.empty()) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger(
        "map_io"), "Map yaml file name starts with '~/' but no HOME variable set. \n"
        << "[INFO] [map_io] User home dir will be not expanded \n");
    return yaml_filename;
  }
  const std::string prefix{home_variable_value};
  return yaml_filename.replace(0, 1, prefix);
}

LoadParameters loadMapYaml(const std::string & yaml_filename)
{
  YAML::Node doc = YAML::LoadFile(expand_user_home_dir_if_needed(yaml_filename, get_home_dir()));
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

  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "resolution: " << load_parameters.resolution);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "origin[0]: " << load_parameters.origin[0]);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "origin[1]: " << load_parameters.origin[1]);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "origin[2]: " << load_parameters.origin[2]);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "free_thresh: " << load_parameters.free_thresh);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(
      "map_io"), "occupied_thresh: " << load_parameters.occupied_thresh);
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("map_io"),
    "mode: " << map_mode_to_string(load_parameters.mode));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "negate: " << load_parameters.negate);

  return load_parameters;
}

void loadMapFromFile(
  const LoadParameters & load_parameters,
  nav_msgs::msg::OccupancyGrid & map)
{
  Magick::InitializeMagick(nullptr);
  nav_msgs::msg::OccupancyGrid msg;

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("map_io"), "Loading image_file: " <<
      load_parameters.image_file_name);
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

  // Convert the image to grayscale
  Magick::Image gray = img;
  gray.type(Magick::GrayscaleType);

  // Prepare grayscale matrix from image
  size_t width = gray.columns();
  size_t height = gray.rows();

  std::vector<uint8_t> buffer(width * height);
  gray.write(0, 0, width, height, "I", Magick::CharPixel, buffer.data());

  // Map the grayscale buffer to an Eigen matrix (row-major layout)
  Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
  gray_matrix(buffer.data(), height, width);

  bool has_alpha = img.matte();

  // Handle different map modes with if else condition
  // Trinary and Scale modes are handled together
  // because they share a lot of code
  // Raw mode is handled separately in else if block
  Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> result(height, width);

  if (load_parameters.mode == MapMode::Trinary || load_parameters.mode == MapMode::Scale) {
    // Convert grayscale to float in range [0.0, 1.0]
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,
      Eigen::RowMajor> normalized = gray_matrix.cast<float>() / 255.0f;

    // Negate the image if specified (e.g. for black=occupied vs. white=occupied convention)
    if (!load_parameters.negate) {
      normalized = (1.0f - normalized.array()).matrix();
    }

    // Compute binary occupancy masks
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> occupied =
      (normalized.array() >= load_parameters.occupied_thresh).cast<uint8_t>();

    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> free =
      (normalized.array() <= load_parameters.free_thresh).cast<uint8_t>();

    // Initialize occupancy grid with UNKNOWN values (-1)
    result.setConstant(nav2_util::OCC_GRID_UNKNOWN);

    // Apply occupied and free cell updates
    result = (occupied.array() > 0).select(nav2_util::OCC_GRID_OCCUPIED, result);
    result = (free.array() > 0).select(nav2_util::OCC_GRID_FREE, result);

    // Handle intermediate (gray) values if in Scale mode
    if (load_parameters.mode == MapMode::Scale) {
      // Create in-between mask
      auto in_between_mask = (normalized.array() > load_parameters.free_thresh) &&
        (normalized.array() < load_parameters.occupied_thresh);

      if (in_between_mask.any()) {
        // Scale in-between values to [0,100] range
        Eigen::ArrayXXf scaled_float = ((normalized.array() - load_parameters.free_thresh) /
          (load_parameters.occupied_thresh - load_parameters.free_thresh)) * 100.0f;

        // Round and cast to int8_t
        Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> scaled_int =
          scaled_float.array().round().cast<int8_t>();

        result = in_between_mask.select(scaled_int, result);
      }
    }

    // Apply alpha transparency mask: mark transparent cells as UNKNOWN
    if (has_alpha) {
      // Allocate buffer only once and map directly to Eigen without extra copy
      std::vector<uint8_t> alpha_buf(width * height);
      img.write(0, 0, width, height, "A", Magick::CharPixel, alpha_buf.data());

      // Map alpha buffer as Eigen::Array for efficient elementwise ops
      Eigen::Map<Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic,
        Eigen::RowMajor>> alpha_array(
        alpha_buf.data(), height, width);

      // Apply mask directly with Eigen::select
      result = (alpha_array < 255).select(nav2_util::OCC_GRID_UNKNOWN, result);
    }

  } else if (load_parameters.mode == MapMode::Raw) {
    // Raw mode: interpret raw image pixel values directly as occupancy values
    result = gray_matrix.cast<int8_t>();

    // Clamp out-of-bound values (outside [-1, 100]) to UNKNOWN (-1)
    auto out_of_bounds = (result.array() < nav2_util::OCC_GRID_FREE) ||
      (result.array() > nav2_util::OCC_GRID_OCCUPIED);

    result = out_of_bounds.select(nav2_util::OCC_GRID_UNKNOWN, result);

  } else {
    // If the map mode is not recognized, throw an error
    throw std::runtime_error("Invalid map mode");
  }

  // Flip image vertically (as ROS expects origin at bottom-left)
  Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic,
    Eigen::RowMajor> flipped = result.colwise().reverse();
  std::memcpy(msg.data.data(), flipped.data(), width * height);

  // Since loadMapFromFile() does not belong to any node, publishing in a system time.
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  msg.info.map_load_time = clock.now();
  msg.header.frame_id = "map";
  msg.header.stamp = clock.now();

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(
      "map_io"), "Read map " << load_parameters.image_file_name
                             << ": " << msg.info.width << " X " << msg.info.height << " map @ "
                             << msg.info.resolution << " m/cell");

  map = msg;
}

LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  nav_msgs::msg::OccupancyGrid & map)
{
  if (yaml_file.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_io"), "YAML file name is empty, can't load!");
    return MAP_DOES_NOT_EXIST;
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "Loading yaml file: " << yaml_file);
  LoadParameters load_parameters;
  try {
    load_parameters = loadMapYaml(yaml_file);
  } catch (YAML::Exception & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "map_io"), "Failed processing YAML file " << yaml_file << " at position (" <<
        e.mark.line << ":" << e.mark.column << ") for reason: " << e.what());
    return INVALID_MAP_METADATA;
  } catch (std::exception & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("map_io"), "Failed to parse map YAML loaded from file " << yaml_file <<
        " for reason: " << e.what());
    return INVALID_MAP_METADATA;
  }
  try {
    loadMapFromFile(load_parameters, map);
  } catch (std::exception & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "map_io"), "Failed to load image file " << load_parameters.image_file_name <<
        " for reason: " << e.what());
    return INVALID_MAP_DATA;
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
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("map_io"), "Map file unspecified. Map will be saved to " <<
        save_parameters.map_file_name << " file");
  }

  // Checking thresholds
  if (save_parameters.occupied_thresh == 0.0) {
    save_parameters.occupied_thresh = 0.65;
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger(
        "map_io"), "Occupied threshold unspecified. Setting it to default value: " <<
        save_parameters.occupied_thresh);
  }
  if (save_parameters.free_thresh == 0.0) {
    save_parameters.free_thresh = 0.25;
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("map_io"), "Free threshold unspecified. Setting it to default value: " <<
        save_parameters.free_thresh);
  }
  if (1.0 < save_parameters.occupied_thresh) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_io"), "Threshold_occupied must be 1.0 or less");
    throw std::runtime_error("Incorrect thresholds");
  }
  if (save_parameters.free_thresh < 0.0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_io"), "Free threshold must be 0.0 or greater");
    throw std::runtime_error("Incorrect thresholds");
  }
  if (save_parameters.occupied_thresh <= save_parameters.free_thresh) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "map_io"), "Threshold_free must be smaller than threshold_occupied");
    throw std::runtime_error("Incorrect thresholds");
  }

  // Checking image format
  if (save_parameters.image_format == "") {
    save_parameters.image_format = save_parameters.mode == MapMode::Scale ? "png" : "pgm";
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("map_io"), "Image format unspecified. Setting it to: " <<
        save_parameters.image_format);
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
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("map_io"), "Requested image format '" << save_parameters.image_format <<
        "' is not one of the recommended formats: " << ss.str());
  }
  const std::string FALLBACK_FORMAT = "png";

  try {
    Magick::CoderInfo info(save_parameters.image_format);
    if (!info.isWritable()) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("map_io"), "Format '" << save_parameters.image_format <<
          "' is not writable. Using '" << FALLBACK_FORMAT << "' instead");
      save_parameters.image_format = FALLBACK_FORMAT;
    }
  } catch (Magick::ErrorOption & e) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger(
        "map_io"), "Format '" << save_parameters.image_format << "' is not usable. Using '" <<
        FALLBACK_FORMAT << "' instead:" << std::endl << e.what());
    save_parameters.image_format = FALLBACK_FORMAT;
  }

  // Checking map mode
  if (
    save_parameters.mode == MapMode::Scale &&
    (save_parameters.image_format == "pgm" ||
    save_parameters.image_format == "jpg" ||
    save_parameters.image_format == "jpeg"))
  {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("map_io"), "Map mode 'scale' requires transparency, but format '" <<
        save_parameters.image_format <<
        "' does not support it. Consider switching image format to 'png'.");
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
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(
      "map_io"), "Received a " << map.info.width << " X " << map.info.height << " map @ " <<
      map.info.resolution << " m/pix");

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
            RCLCPP_ERROR_STREAM(
              rclcpp::get_logger(
                "map_io"), "Map mode should be Trinary, Scale or Raw");
            throw std::runtime_error("Invalid map mode");
        }
        image.pixelColor(x, y, pixel);
      }
    }

    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("map_io"),
      "Writing map occupancy data to " << mapdatafile);
    image.write(mapdatafile);
  }

  std::string mapmetadatafile = save_parameters.map_file_name + ".yaml";
  {
    std::ofstream yaml(mapmetadatafile);

    geometry_msgs::msg::Quaternion orientation = map.info.origin.orientation;
    tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    const int file_name_index = mapdatafile.find_last_of("/\\");
    std::string image_name = mapdatafile.substr(file_name_index + 1);

    YAML::Emitter e;
    e << YAML::Precision(7);
    e << YAML::BeginMap;
    e << YAML::Key << "image" << YAML::Value << image_name;
    e << YAML::Key << "mode" << YAML::Value << map_mode_to_string(save_parameters.mode);
    e << YAML::Key << "resolution" << YAML::Value << to_string_with_precision(map.info.resolution,
        3);
    e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq <<
      to_string_with_precision(map.info.origin.position.x, 3) <<
      to_string_with_precision(map.info.origin.position.y, 3) << yaw << YAML::EndSeq;
    e << YAML::Key << "negate" << YAML::Value << 0;

    if (save_parameters.mode == MapMode::Trinary) {
      // For Trinary mode, the thresholds depend on the pixel values in the saved map,
      // not on the thresholds used to threshold the map.
      // As these values are fixed above, the thresholds must also be fixed to separate the
      // pixel values into occupied, free and unknown.
      e << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
      e << YAML::Key << "free_thresh" << YAML::Value << 0.196;
    } else {
      e << YAML::Key << "occupied_thresh" << YAML::Value <<
        to_string_with_precision(save_parameters.occupied_thresh, 3);
      e << YAML::Key << "free_thresh" << YAML::Value <<
        to_string_with_precision(save_parameters.free_thresh, 3);
    }

    if (!e.good()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("map_io"), "YAML writer failed with an error " << e.GetLastError() <<
          ". The map metadata may be invalid.");
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "Writing map metadata to " << mapmetadatafile);
    std::ofstream(mapmetadatafile) << e.c_str();
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("map_io"), "Map saved");
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
  } catch (std::exception & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("map_io"),
      "Failed to write map for reason: " << e.what());
    return false;
  }
  return true;
}

std::string to_string_with_precision(double value, int precision)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;

  return out.str();
}

}  // namespace nav2_map_server
