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
// Created by shivam on 7/10/20.
//

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
void modifyMsgFields(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::vector<pcl::PCLPointField> & fields);

void pclToMsg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::shared_ptr<pcl::PCLPointCloud2> & cloud);

void modifyPclFields(
  std::vector<pcl::PCLPointField> & fields,
  const sensor_msgs::msg::PointCloud2 & msg);

void msgToPcl(
  std::shared_ptr<pcl::PCLPointCloud2> & cloud,
  const sensor_msgs::msg::PointCloud2 & msg);

bool ends_with(std::string const & value, std::string const & ending);
}  // namespace map_3d
}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_3D__PCL_HELPER_HPP_
