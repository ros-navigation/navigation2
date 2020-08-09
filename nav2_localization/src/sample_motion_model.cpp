#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/plugins/sample_motion_model_plugins.hpp"

namespace nav2_localization_plugins
{

geometry_msgs::msg::Pose DummyMotionSampler::getMostLikelyPose(
        const nav_msgs::msg::Odometry& prev_odom,
        const nav_msgs::msg::Odometry& curr_odom,
        const geometry_msgs::msg::Pose& prev_pose)
{
    geometry_msgs::msg::Pose dummy_pose{};
    return dummy_pose;
}

void DummyMotionSampler::configure(
        const double& alpha1,
        const double& alpha2,
        const double& alpha3,
        const double& alpha4,
        const double& alpha5)
{
    alpha1_ = alpha1;
    alpha2_ = alpha2;
    alpha3_ = alpha3;
    alpha4_ = alpha4;
    alpha5_ = alpha5;
}

} // nav2_localization_plugins

PLUGINLIB_EXPORT_CLASS(nav2_localization_plugins::DummyMotionSampler, nav2_localization_base::SampleMotionModel)