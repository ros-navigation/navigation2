#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/plugins/sample_motion_model_plugins.hpp"

namespace nav2_localization
{

geometry_msgs::msg::Pose DummyMotionSampler::getMostLikelyPose(
        const nav_msgs::msg::Odometry& prev_odom,
        const nav_msgs::msg::Odometry& curr_odom,
        const geometry_msgs::msg::Pose& prev_pose)
{
    geometry_msgs::msg::Pose dummy_pose{};
    return dummy_pose;
}

void DummyMotionSampler::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
{
    node_ = node;
}

void DummyMotionSampler::activate()
{

}

void DummyMotionSampler::deactivate()
{

}

void DummyMotionSampler::cleanup()
{
    
}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::DummyMotionSampler, nav2_localization::SampleMotionModel)
