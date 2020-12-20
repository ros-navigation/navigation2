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

/*Helper functions for PCL<->sensor_msg conversions*/

#ifndef NAV2_MAP_SERVER__MAP_3D__PCL_HELPER_HPP_
#define NAV2_MAP_SERVER__MAP_3D__PCL_HELPER_HPP_

#include <vector>
#include <memory>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl/PCLPointField.h"
#include "pcl_conversions/pcl_conversions.h"

namespace nav2_map_server
{

namespace  map_3d
{

/**
 * @brief Modifies message fields that describes how
 * the pointcloud data is arranged and type of each field.
 * @param msg
 * @param fields
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

}  // namespace map_3d

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_3D__PCL_HELPER_HPP_
