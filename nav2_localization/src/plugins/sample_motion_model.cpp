#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/plugins/sample_motion_model_plugins.hpp"

namespace nav2_localization
{

bool DiffDriveOdomMotionModel::SampleFrom(Sample<geometry_msgs::msg::TransformStamped>& one_sample, 
    const SampleMthd method,
    void * args) const
{
    return true;
}

void DiffDriveOdomMotionModel::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
{
    node_ = node;
}

void DiffDriveOdomMotionModel::activate()
{

}

void DiffDriveOdomMotionModel::deactivate()
{

}

void DummyMotionDiffDriveOdomMotionModelSampler::cleanup()
{
    
}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::DiffDriveOdomMotionModel, nav2_localization::SampleMotionModel)
