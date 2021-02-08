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

#include "nav2_map_server/map_3d/pcl_helper.hpp"

#include <iostream>
#include <vector>
#include <string>

namespace nav2_map_server
{

namespace map_3d
{

void modifyMsgFields(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::vector<pcl::PCLPointField> & fields)
{
  msg.fields.clear();
  for (auto & field : fields) {
    sensor_msgs::msg::PointField new_field;
    new_field.datatype = field.datatype;
    new_field.name = field.name;
    new_field.count = field.count;
    new_field.offset = field.offset;
    msg.fields.push_back(new_field);
  }
  std::cout << "[DEBUG] [pcl_helper]: Message field modification done" << std::endl;
}

void pclToMsg(
  sensor_msgs::msg::PointCloud2 & msg,
  const pcl::PCLPointCloud2::Ptr & cloud)
{
  msg.data.clear();
  modifyMsgFields(msg, cloud->fields);
  msg.data = cloud->data;
  msg.point_step = cloud->point_step;
  msg.row_step = cloud->row_step;
  msg.width = cloud->width;
  msg.height = cloud->height;
  msg.is_bigendian = cloud->is_bigendian;
  msg.is_dense = cloud->is_dense;
  msg.header = pcl_conversions::fromPCL(cloud->header);
  std::cout << "[DEBUG] [pcl_helper]: PCL to message conversion done" << std::endl;
}

void modifyPclFields(
  std::vector<pcl::PCLPointField> & fields,
  const sensor_msgs::msg::PointCloud2 & msg)
{
  fields.clear();
  for (auto & field : msg.fields) {
    pcl::PCLPointField new_field;
    new_field.datatype = field.datatype;
    new_field.name = field.name;
    new_field.count = field.count;
    new_field.offset = field.offset;
    fields.push_back(new_field);
  }
  std::cout << "[DEBUG] [pcl_helper]: PCL field modification done" << std::endl;
}

void msgToPcl(
  pcl::PCLPointCloud2::Ptr & cloud,
  const sensor_msgs::msg::PointCloud2 & msg)
{
  cloud->data.clear();
  modifyPclFields(cloud->fields, msg);
  cloud->data = msg.data;
  cloud->point_step = msg.point_step;
  cloud->row_step = msg.row_step;
  cloud->width = msg.width;
  cloud->height = msg.height;
  cloud->is_bigendian = msg.is_bigendian;
  cloud->is_dense = msg.is_dense;
  cloud->header = pcl_conversions::toPCL(msg.header);
  std::cout << "[DEBUG] [pcl_helper]: message to PCL conversion done" << std::endl;
}

bool ends_with(std::string const & value, std::string const & ending)
{
  if (ending.size() > value.size()) {
    return false;
  }
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void viewPoint2Pose(
  geometry_msgs::msg::Pose & origin,
  const Eigen::Vector4f & position,
  const Eigen::Quaternionf & orientation)
{
  // Put position information to origin
  origin.position.x = static_cast<double>(position[0]);
  origin.position.y = static_cast<double>(position[1]);
  origin.position.z = static_cast<double>(position[2]);

  // Put orientation information to origin
  origin.orientation.w = static_cast<double>(orientation.w());
  origin.orientation.x = static_cast<double>(orientation.x());
  origin.orientation.y = static_cast<double>(orientation.y());
  origin.orientation.z = static_cast<double>(orientation.z());
}

void pose2ViewPoint(
  Eigen::Vector4f & position,
  Eigen::Quaternionf & orientation,
  const geometry_msgs::msg::Pose & origin)
{
  // Update center information of viewPoint
  position[0] = static_cast<float>(origin.position.x);
  position[1] = static_cast<float>(origin.position.y);
  position[2] = static_cast<float>(origin.position.z);

  // Update center information of viewPoint
  orientation.w() = static_cast<float>(origin.orientation.w);
  orientation.x() = static_cast<float>(origin.orientation.x);
  orientation.y() = static_cast<float>(origin.orientation.y);
  orientation.z() = static_cast<float>(origin.orientation.z);
}

}  // namespace map_3d

}  // namespace nav2_map_server
