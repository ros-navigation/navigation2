#include <random>
#include "pdfs/diff_drive_odom_sample_pdf.hpp"
#include "tf2/utils.h"

namespace nav2_localization
{

DiffDriveOdomSamplePdf::DiffDriveOdomSamplePdf(const double &a1,
                                               const double &a2,
                                               const double &a3,
                                               const double &a4,
                                               const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) :
                                               alpha1_(a1), alpha2_(a2), alpha3_(a3), alpha4_(a4), node_(node)
{}

bool DiffDriveOdomSamplePdf::SampleFrom(
    BFL::Sample<geometry_msgs::msg::TransformStamped>& one_sample, 
    const int method,
    void * args) const
{
    geometry_msgs::msg::TransformStamped prev_odom = ConditionalArgumentGet(0);
    geometry_msgs::msg::TransformStamped curr_odom = ConditionalArgumentGet(1);
    geometry_msgs::msg::TransformStamped prev_pose = ConditionalArgumentGet(2);
    geometry_msgs::msg::TransformStamped most_likely_pose;

    double x_bar_prime = curr_odom.transform.translation.x;
    double y_bar_prime = curr_odom.transform.translation.y;
    double theta_bar_prime = tf2::getYaw(curr_odom.transform.rotation);
    
    double x_bar = prev_odom.transform.translation.x;
    double y_bar = prev_odom.transform.translation.y;
    double theta_bar = tf2::getYaw(prev_odom.transform.rotation);

    double x = prev_pose.transform.translation.x;
    double y = prev_pose.transform.translation.y;
    double theta = tf2::getYaw(prev_pose.transform.rotation);

    double delta_rot_1 = atan2(y_bar_prime-y_bar, x_bar_prime-x_bar) - theta_bar;
    if(isnan(delta_rot_1) || isinf(delta_rot_1))
    {
        RCLCPP_ERROR(node_->get_logger(), "delta_rot_1 is NAN or INF");
        delta_rot_1 = 0.0; // TODO: consider a different value
    }
    double delta_trans = hypot(x_bar_prime-x_bar, y_bar_prime-y_bar);
    double delta_rot_2 = theta_bar_prime - theta_bar - delta_rot_1;

    std::random_device device_;
    std::mt19937 generator_(device_());

    std::normal_distribution<double> delta_rot_1_noise_dist(0.0, sqrt(alpha1_*delta_rot_1 + alpha2_*delta_trans));
    double delta_rot_1_hat = delta_rot_1 - delta_rot_1_noise_dist(generator_);

    std::normal_distribution<double> delta_trans_noise_dist(0.0, sqrt(alpha3_*delta_trans + alpha4_*(delta_rot_1+delta_rot_2)));
    double delta_trans_hat = delta_trans - delta_trans_noise_dist(generator_);

    std::normal_distribution<double> delta_rot_2_noise_dist(0.0, sqrt(alpha1_*delta_rot_2 + alpha2_*delta_trans));
    double delta_rot_2_hat = delta_rot_2 - delta_rot_2_noise_dist(generator_);

    most_likely_pose.transform.translation.x = x + delta_trans_hat*cos(theta + delta_rot_1_hat);
    most_likely_pose.transform.translation.y = y + delta_trans_hat*sin(theta + delta_rot_1_hat);

    tf2::Quaternion theta_prime_quat;
    double theta_prime = theta + delta_rot_1_hat + delta_rot_2_hat;
    theta_prime_quat.setEuler(theta_prime, 0.0, 0.0);

    most_likely_pose.transform.rotation = tf2::toMsg(theta_prime_quat);

    one_sample.ValueSet(most_likely_pose);

    return true;
}

} // namespace nav2_localization
