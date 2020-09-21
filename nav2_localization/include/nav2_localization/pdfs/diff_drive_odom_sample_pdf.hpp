#ifndef NAV2_LOCALIZATION__DIFF_DRIVE_ODOM_SAMPLE_PDF_HPP_
#define NAV2_LOCALIZATION__DIFF_DRIVE_ODOM_SAMPLE_PDF_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <pdf/conditionalpdf.h>

namespace nav2_localization
{
class DiffDriveOdomSamplePdf : public BFL::ConditionalPdf
    <geometry_msgs::msg::TransformStamped, geometry_msgs::msg::TransformStamped>
{
public:
    DiffDriveOdomSamplePdf(const double &a1, const double &a2, const double &a3, const double &a4,
                           const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);
    bool SampleFrom(BFL::Sample<geometry_msgs::msg::TransformStamped>& one_sample,
        const int method = DEFAULT,
        void * args = NULL) const;

private:
    double alpha1_;
    double alpha2_;
    double alpha3_;
    double alpha4_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};
} // namespace nav2_localization

#endif // NAV2_LOCALIZATION__DIFF_DRIVE_ODOM_SAMPLE_PDF_HPP_