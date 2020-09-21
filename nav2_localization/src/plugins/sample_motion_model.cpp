#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/plugins/sample_motion_model_plugins.hpp"
#include "pdfs/diff_drive_odom_sample_pdf.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include <cmath>
#include <random>

namespace nav2_localization
{

void DiffDriveOdomMotionModel::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
{
    node_ = node;

    // set noise parameters
    node_->get_parameter("alpha1", alpha1_);
    node_->get_parameter("alpha2", alpha2_);
    node_->get_parameter("alpha3", alpha3_);
    node_->get_parameter("alpha4", alpha4_);

    auto system_pdf = std::make_unique<DiffDriveOdomSamplePdf>(alpha1_, alpha2_, alpha3_, alpha4_, node_);
    this->SystemPdfSet(system_pdf.get());
}

void DiffDriveOdomMotionModel::activate()
{

}

void DiffDriveOdomMotionModel::deactivate()
{

}

void DiffDriveOdomMotionModel::cleanup()
{
    
}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::DiffDriveOdomMotionModel, nav2_localization::SampleMotionModel)
