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

/*Helper functions for PCL<->sensor_msg conversions*/

#ifndef NAV2_MAP_SERVER__MAP_3D__PCL_HELPER_HPP_
#define NAV2_MAP_SERVER__MAP_3D__PCL_HELPER_HPP_

#include <vector>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "pcl/PCLPointField.h"
#include "pcl_conversions/pcl_conversions.h"
#include "Eigen/Core"

namespace nav2_map_server
{

namespace  map_3d
{

/**
 * @brief Modifies message fields that describes how
 * the pointcloud data is arranged and type of each field.
 * @param msg Output PointCloud2 sensor message with modified fields.
 * @param fields Input PCLPointFields indicating field types used in pcd-v0.7
 */
void modifyMsgFields(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::vector<pcl::PCLPointField> & fields);

/**
 * @brief Converts map from pcl::PointCloud2 to sensor_msgs::msg::PointCloud2
 * @param msg message object to be changed according to the input pointcloud
 * @param cloud pointcloud to be converted in message object
 */
void pclToMsg(
  sensor_msgs::msg::PointCloud2 & msg,
  const pcl::PCLPointCloud2::Ptr & cloud);

/**
 * @brief Modifies the pointcloud2 fields in pcl scope
 * @param fields pointfields modified according to incoming message
 * @param msg message containing the pointfields to be converted
 */
void modifyPclFields(
  std::vector<pcl::PCLPointField> & fields,
  const sensor_msgs::msg::PointCloud2 & msg);

/**
 * @brief Converts pointcloud sensor_msgs::msg::PointCloud2 to  from pcl::PointCloud2
 * @param cloud pointcloud object to be changed according to the input message
 * @param msg message to be converted in pointcloud object
 */
void msgToPcl(
  pcl::PCLPointCloud2::Ptr & cloud,
  const sensor_msgs::msg::PointCloud2 & msg);

/**
 * @brief Helper function to match input string with the desired ending
 * @param value input string
 * @param ending desired ending
 * @return true or false
 */
bool ends_with(std::string const & value, std::string const & ending);

/**
 * @brief Converts position and orientation from PCL to geometry_msg format
 * @param origin desired Pose in geonetry_msg format
 * @param position input Eigen::Vector4f from pcl
 * @param orientation input Eigen::Quaternionf from pcl
 */
void viewPoint2Pose(
  geometry_msgs::msg::Pose & origin,
  const Eigen::Vector4f & position,
  const Eigen::Quaternionf & orientation);

/**
 * @brief Converts origin info from geometry_msgs format to pcl position and orientation
 * @param position desired Eigen::Vector4f
 * @param orientation desired Eigen::Quaternionf
 * @param origin input geometry_msgs::msg::Pose
 */
// void pose2ViewPoint(
//   Eigen::Vector4f & position,
//   Eigen::Quaternionf & orientation,
//   const geometry_msgs::msg::Pose & origin);

}  // namespace map_3d

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_3D__PCL_HELPER_HPP_
