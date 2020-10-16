#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/plugins/sample_motion_model_plugins.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include <cmath>
#include <random>

namespace nav2_localization
{

bool DiffDriveOdomMotionModelPDF::SampleFrom(	
    BFL::Sample<geometry_msgs::msg::TransformStamped>& one_sample, 	
    const BFL::SampleMthd method,	
    void * args) const	
{
    // Get conditional arguments	
    geometry_msgs::msg::TransformStamped prev_odom = ConditionalArgumentGet(0);	
    geometry_msgs::msg::TransformStamped curr_odom = ConditionalArgumentGet(1);	
    geometry_msgs::msg::TransformStamped prev_pose = ConditionalArgumentGet(2);	

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
        RCLCPP_ERROR(node_->get_logger(), "deltta_rot_1 is NAN or INF");	
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

    geometry_msgs::msg::TransformStamped most_likely_pose;	
    most_likely_pose.transform.translation.x = x + delta_trans_hat*cos(theta + delta_rot_1_hat);	
    most_likely_pose.transform.translation.y = y + delta_trans_hat*sin(theta + delta_rot_1_hat);	

    tf2::Quaternion theta_prime_quat;	
    double theta_prime = theta + delta_rot_1_hat + delta_rot_2_hat;	
    theta_prime_quat.setEuler(theta_prime, 0.0, 0.0);	

    most_likely_pose.transform.rotation = tf2::toMsg(theta_prime_quat);	

    one_sample.ValueSet(most_likely_pose);	

    return true;	
}

void DiffDriveOdomMotionModelPDF::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
{
    node_ = node;

    // set noise parameters
    node_->get_parameter("alpha1", alpha1_);
    node_->get_parameter("alpha2", alpha2_);
    node_->get_parameter("alpha3", alpha3_);
    node_->get_parameter("alpha4", alpha4_);
}

void DiffDriveOdomMotionModelPDF::activate()
{

}

void DiffDriveOdomMotionModelPDF::deactivate()
{

}

void DiffDriveOdomMotionModelPDF::cleanup()
{
    
}

} // nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::DiffDriveOdomMotionModelPDF, nav2_localization::SampleMotionModelPDF)
