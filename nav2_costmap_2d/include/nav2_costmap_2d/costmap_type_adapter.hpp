#ifndef NAV2_COSTMAP_2D__COSTMAP_TYPE_ADAPTER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_TYPE_ADAPTER_HPP_
#include <type_traits>
#include <algorithm>
#include <memory>

#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp/type_adapter.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"

namespace nav2_costmap_2d
{

// In-process type. On the wire it masquerades as nav2_msgs::msg::Costmap.
struct Costmap2DStamped
{
  std_msgs::msg::Header header;
  nav2_msgs::msg::CostmapMetaData metadata;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
};

}  // namespace nav2_costmap_2d

namespace rclcpp
{

template<>
struct TypeAdapter<nav2_costmap_2d::Costmap2DStamped, nav2_msgs::msg::Costmap>
{
  using is_specialized = std::true_type;
  using custom_type = nav2_costmap_2d::Costmap2DStamped;
  using ros_message_type = nav2_msgs::msg::Costmap;

  static void convert_to_ros_message(const custom_type & src, ros_message_type & dst)
  {
    dst.header = src.header;
    dst.metadata = src.metadata;

    if (!src.costmap) {
      dst.data.clear();
      return;
    }

    const size_t size_x = static_cast<size_t>(src.costmap->getSizeInCellsX());
    const size_t size_y = static_cast<size_t>(src.costmap->getSizeInCellsY());
    const size_t n = size_x * size_y;

    dst.data.resize(n);
    const unsigned char * p = src.costmap->getCharMap();
    std::copy(p, p + n, dst.data.begin());
  }

  static void convert_to_custom(const ros_message_type & src, custom_type & dst)
  {
    dst.header = src.header;
    dst.metadata = src.metadata;

    const unsigned int size_x = src.metadata.size_x;
    const unsigned int size_y = src.metadata.size_y;

    auto cm = std::make_shared<nav2_costmap_2d::Costmap2D>(
      size_x, size_y,
      src.metadata.resolution,
      src.metadata.origin.position.x,
      src.metadata.origin.position.y);

    const size_t expected = static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
    const size_t n = std::min(expected, src.data.size());

    unsigned char * out = cm->getCharMap();
    std::copy(src.data.begin(), src.data.begin() + n, out);

    if (n < expected) {
      std::fill(out + n, out + expected, 0u);
    }

    dst.costmap = std::move(cm);
  }
};

}  // namespace rclcpp

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nav2_costmap_2d::Costmap2DStamped,
  nav2_msgs::msg::Costmap);

#endif  // NAV2_COSTMAP_2D__COSTMAP_TYPE_ADAPTER_HPP_
