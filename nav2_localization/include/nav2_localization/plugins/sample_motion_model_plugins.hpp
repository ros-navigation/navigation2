#ifndef NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_PLUGINS_HPP_
#define NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_PLUGINS_HPP_

#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{
class DiffDriveOdomMotionModel : public nav2_localization::SampleMotionModel
{
public:
    DiffDriveOdomMotionModel(){}

    bool SampleFrom(Sample<geometry_msgs::msg::TransformStamped>& one_sample,
        const SampleMthd method = SampleMthd::DEFAULT,
        void * args = NULL) const;

    void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);
    void activate();
    void deactivate();
    void cleanup();
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_PLUGINS_HPP_
