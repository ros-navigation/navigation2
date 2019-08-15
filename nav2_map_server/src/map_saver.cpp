/*
 * Copyright 2019 Rover Robotics
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "nav2_map_server/map_saver.hpp"

#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Magick++.h"
#include "nav2_map_server/map_mode.hpp"
#include "nav_msgs/msg/occupancy_grid.h"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

namespace nav2_map_server
{
MapSaver::MapSaver(const rclcpp::NodeOptions & options)
: Node("map_saver", options), save_next_map_promise{}
{
  Magick::InitializeMagick(nullptr);
  {
    mapname_ = declare_parameter("output_file_no_ext", "map");
    if (mapname_.empty()) {
      throw std::runtime_error("Map name not provided");
    }
    threshold_occupied_ = declare_parameter("threshold_occupied", 65);
    if (100 < threshold_occupied_) {
      throw std::runtime_error("Threshold_occupied must be 100 or less");
    }
    threshold_free_ = declare_parameter("threshold_free", 25);
    if (threshold_free_ < 0) {
      throw std::runtime_error("Free threshold must be 0 or greater");
    }
    if (threshold_occupied_ <= threshold_free_) {
      throw std::runtime_error("Threshold_free must be smaller than threshold_occupied");
    }

    std::string mode_str = declare_parameter("map_mode", "trinary");
    try {
      map_mode = map_mode_from_string(mode_str);
    } catch (std::invalid_argument &) {
      map_mode = MapMode::Trinary;
      RCLCPP_WARN(
        get_logger(), "Map mode parameter not recognized: '%s', using default value (trinary)",
        mode_str.c_str());
    }

    image_format = declare_parameter("image_format", map_mode == MapMode::Scale ? "png" : "pgm");
    std::transform(
      image_format.begin(), image_format.end(), image_format.begin(),
      [](unsigned char c) {return std::tolower(c);});
    const std::vector<std::string> BLESSED_FORMATS{"bmp", "pgm", "png"};
    if (
      std::find(BLESSED_FORMATS.begin(), BLESSED_FORMATS.end(), image_format) ==
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
      RCLCPP_WARN(
        get_logger(), "Requested image format '%s' is not one of the recommended formats: %s",
        image_format.c_str(), ss.str().c_str());
    }
    const std::string FALLBACK_FORMAT = "png";

    try {
      Magick::CoderInfo info(image_format);
      if (!info.isWritable()) {
        RCLCPP_WARN(
          get_logger(), "Format '%s' is not writable. Using '%s' instead",
          image_format.c_str(), FALLBACK_FORMAT.c_str());
        image_format = FALLBACK_FORMAT;
      }
    } catch (Magick::ErrorOption & e) {
      RCLCPP_WARN(
        get_logger(), "Format '%s' is not usable. Using '%s' instead:\n%s",
        image_format.c_str(), FALLBACK_FORMAT.c_str(), e.what());
      image_format = FALLBACK_FORMAT;
    }
    if (
      map_mode == MapMode::Scale &&
      (image_format == "pgm" || image_format == "jpg" || image_format == "jpeg"))
    {
      RCLCPP_WARN(
        get_logger(),
        "Map mode 'scale' requires transparency, but format '%s' does not support it. Consider "
        "switching image format to 'png'.",
        image_format.c_str());
    }

    RCLCPP_INFO(get_logger(), "Waiting for the map");
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::SystemDefaultsQoS(),
      std::bind(&MapSaver::mapCallback, this, std::placeholders::_1));
  }
}

void MapSaver::try_write_map_to_file(const nav_msgs::msg::OccupancyGrid & map)
{
  auto logger = get_logger();
  RCLCPP_INFO(
    logger, "Received a %d X %d map @ %.3f m/pix", map.info.width, map.info.height,
    map.info.resolution);

  std::string mapdatafile = mapname_ + "." + image_format;
  {
    // should never see this color, so the initialization value is just for debugging
    Magick::Image image({map.info.width, map.info.height}, "red");

    // In scale mode, we need the alpha (matte) channel. Else, we don't.
    // NOTE: GraphicsMagick seems to have trouble loading the alpha channel when saved with
    // Magick::GreyscaleMatte, so we use TrueColorMatte instead.
    image.type(map_mode == MapMode::Scale ? Magick::TrueColorMatteType : Magick::GrayscaleType);

    // Since we only need to support 100 different pixel levels, 8 bits is fine
    image.depth(8);

    for (size_t y = 0; y < map.info.height; y++) {
      for (size_t x = 0; x < map.info.width; x++) {
        int8_t map_cell = map.data[map.info.width * (map.info.height - y - 1) + x];

        Magick::Color pixel;

        switch (map_mode) {
          case MapMode::Trinary:
            if (map_cell < 0 || 100 < map_cell) {
              pixel = Magick::ColorGray(205 / 255.0);
            } else if (map_cell <= threshold_free_) {
              pixel = Magick::ColorGray(254 / 255.0);
            } else if (threshold_occupied_ <= map_cell) {
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
            throw std::runtime_error("Invalid map mode");
        }
        image.pixelColor(x, y, pixel);
      }
    }

    RCLCPP_INFO(logger, "Writing map occupancy data to %s", mapdatafile.c_str());
    image.write(mapdatafile);
  }

  std::string mapmetadatafile = mapname_ + ".yaml";
  {
    std::ofstream yaml(mapmetadatafile);

    geometry_msgs::msg::Quaternion orientation = map.info.origin.orientation;
    tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    YAML::Emitter e;
    e << YAML::Precision(3);
    e << YAML::BeginMap;
    e << YAML::Key << "image" << YAML::Value << mapdatafile;
    e << YAML::Key << "mode" << YAML::Value << map_mode_to_string(map_mode);
    e << YAML::Key << "resolution" << YAML::Value << map.info.resolution;
    e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq << map.info.origin.position.x <<
      map.info.origin.position.y << yaw << YAML::EndSeq;
    e << YAML::Key << "negate" << YAML::Value << 0;
    e << YAML::Key << "occupied_thresh" << YAML::Value << threshold_occupied_ / 100.0;
    e << YAML::Key << "free_thresh" << YAML::Value << threshold_free_ / 100.0;

    if (!e.good()) {
      RCLCPP_WARN(
        logger, "YAML writer failed with an error %s. The map metadata may be invalid.",
        e.GetLastError().c_str());
    }

    RCLCPP_INFO(logger, "Writing map metadata to %s", mapmetadatafile.c_str());
    std::ofstream(mapmetadatafile) << e.c_str();
  }
  RCLCPP_INFO(logger, "Map saved");
}

void MapSaver::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  auto current_promise = std::move(save_next_map_promise);
  save_next_map_promise = std::promise<void>();

  try {
    try_write_map_to_file(*map);
    current_promise.set_value();
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to write map for reason: %s", e.what());
    current_promise.set_exception(std::current_exception());
  }
}
}  // namespace nav2_map_server
