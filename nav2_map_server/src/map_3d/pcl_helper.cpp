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

#include "nav2_map_server_3d/pcl_helper.hpp"

#include <vector>
#include <memory>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl/PCLPointField.h"
#include "pcl_conversions/pcl_conversions.h"

namespace nav2_map_server
{
namespace nav2_map_server_3d
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
}

void pclToMsg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::shared_ptr<pcl::PCLPointCloud2> & cloud)
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
}

void msgToPcl(
  std::shared_ptr<pcl::PCLPointCloud2> & cloud,
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
}

}  // namespace map_3d
}  // namespace nav2_map_server
