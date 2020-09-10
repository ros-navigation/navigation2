#ifndef NAV2_LOCALIZATION__ACTION_HPP_
#define NAV2_LOCALIZATION__ACTION_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace nav2_localization
{
struct Action
{
    geometry_msgs::msg::TransformStamped prev_odom;
    geometry_msgs::msg::TransformStamped curr_odom;
};
}

#endif // NAV2_LOCALIZATION__ACTION_HPP_