// Copyright (c) 2026 Dexory

#ifndef NAV2_COLLISION_MONITOR__TRIGGERING_CLOUD_HPP_
#define NAV2_COLLISION_MONITOR__TRIGGERING_CLOUD_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

class TriggeringCloud
{
public:
  void configure(
    const std::vector<std::string> & sources, const std::vector<std::string> & polygons)
  {
    sources_.clear();
    polygons_.clear();
    for (size_t index = 0; index < sources.size(); ++index) {
      sources_.emplace(sources[index], index);
    }
    for (size_t index = 0; index < polygons.size(); ++index) {
      polygons_.emplace(polygons[index], index);
    }
  }

  static sensor_msgs::msg::PointCloud2 create(const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.is_dense = true;
    const uint16_t endian = 1;
    cloud.is_bigendian = *reinterpret_cast<const uint8_t *>(&endian) == 0;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    using Field = sensor_msgs::msg::PointField;
    modifier.setPointCloud2Fields(
      6, "x", 1, Field::FLOAT32, "y", 1, Field::FLOAT32, "z", 1, Field::FLOAT32,
      "source_id", 1, Field::UINT32, "polygon_id", 1, Field::UINT32,
      "action_type", 1, Field::UINT32);
    return cloud;
  }

  void append(
    sensor_msgs::msg::PointCloud2 & cloud, const std::vector<Point> & points,
    const std::string & polygon, ActionType action) const
  {
    if (points.empty()) {
      return;
    }
    const uint32_t polygon_index = lookup(polygons_, polygon);
    const size_t offset = cloud.width;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.resize(offset + points.size());
    sensor_msgs::PointCloud2Iterator<float> xpos(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> ypos(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> zpos(cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> source_id(cloud, "source_id");
    sensor_msgs::PointCloud2Iterator<uint32_t> polygon_id(cloud, "polygon_id");
    sensor_msgs::PointCloud2Iterator<uint32_t> action_type(cloud, "action_type");
    xpos += offset;
    ypos += offset;
    zpos += offset;
    source_id += offset;
    polygon_id += offset;
    action_type += offset;
    size_t kept = offset;
    for (const auto & point : points) {
      const float coord_x = static_cast<float>(point.x);
      const float coord_y = static_cast<float>(point.y);
      const float coord_z = static_cast<float>(point.z);
      if (!std::isfinite(coord_x) || !std::isfinite(coord_y) || !std::isfinite(coord_z)) {
        continue;
      }
      *xpos = coord_x;
      *ypos = coord_y;
      *zpos = coord_z;
      *source_id = lookup(sources_, point.source);
      *polygon_id = polygon_index;
      *action_type = static_cast<uint32_t>(action);
      ++xpos;
      ++ypos;
      ++zpos;
      ++source_id;
      ++polygon_id;
      ++action_type;
      ++kept;
    }
    modifier.resize(kept);
  }

private:
  static uint32_t lookup(
    const std::unordered_map<std::string, uint32_t> & mapping, const std::string & name)
  {
    const auto found = mapping.find(name);
    return found == mapping.end() ? std::numeric_limits<uint32_t>::max() : found->second;
  }

  std::unordered_map<std::string, uint32_t> sources_;
  std::unordered_map<std::string, uint32_t> polygons_;
};

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__TRIGGERING_CLOUD_HPP_
