#ifndef NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_BASE_HPP_
#define NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_BASE_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{
/**
 * @class SampleMotionModelPDF
 * @brief Abstract interface for sample motion model to adhere to with pluginlib
 */

// ConditionalPdf template paramteres:
// - Var: the most likely pose (x_t)
// - CondArg: the previous pose (x_t-1), the current odom (x_bar_t) and the previous pose
class SampleMotionModel
{   
public:
    using Ptr = std::shared_ptr<nav2_localization::SampleMotionModel>;

    virtual geometry_msgs::msg::TransformStamped getMostLikelyPose(
        const geometry_msgs::msg::TransformStamped& prev_odom,
        const geometry_msgs::msg::TransformStamped& curr_odom,
        const geometry_msgs::msg::TransformStamped& prev_pose);

    virtual void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) = 0;

    virtual void activate() = 0;

    virtual void deactivate() = 0;

    virtual void cleanup() = 0;

protected:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};  
} // nav2_localization

#endif // NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_BASE_HPP_
