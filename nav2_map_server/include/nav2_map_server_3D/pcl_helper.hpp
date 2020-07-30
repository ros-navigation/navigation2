//
// Created by shivam on 7/10/20.
//

#ifndef NAV2_MAP_SERVER_INCLUDE_NAV2_MAP_SERVER_PCL_HELPER_HPP_
#define NAV2_MAP_SERVER_INCLUDE_NAV2_MAP_SERVER_PCL_HELPER_HPP_

#include <vector>
#include <memory>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl/PCLPointField.h"
#include "pcl_conversions/pcl_conversions.h"

namespace nav2_map_server {
namespace nav2_map_server_3D {

void ModifyMsgFields(sensor_msgs::msg::PointCloud2 &msg, const std::vector<pcl::PCLPointField> &fields);

void PclToMsg(sensor_msgs::msg::PointCloud2 &msg, const std::shared_ptr<pcl::PCLPointCloud2> &cloud);

void ModifyPclFields(std::vector<pcl::PCLPointField> &fields, const sensor_msgs::msg::PointCloud2 &msg);

void MsgToPcl(std::shared_ptr<pcl::PCLPointCloud2> &cloud, const sensor_msgs::msg::PointCloud2 &msg);
} // namespace nav2_map_server_3D
} // namespace nav2_map_server
#endif //NAV2_MAP_SERVER_INCLUDE_NAV2_MAP_SERVER_PCL_HELPER_HPP_
