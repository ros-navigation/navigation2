#ifndef NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_BASE_HPP_
#define NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_BASE_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{

/**
 * @class SampleMotionModel
 * @brief Abstract interface for sample motion model to adhere to with pluginlib
 */
class SampleMotionModel
{   
public:
    SampleMotionModel(){}

    using Ptr = std::shared_ptr<nav2_localization::SampleMotionModel>;

    /**
     * @brief calculates the most likely change in pose based on the change in odometry
     * @param prev_odom The odometry at the previous time step
     * @param curr_odom The odometry at the current time step
     * @param prev_pose The pose at the previous time step
     * @return most likely current pose
     */
    virtual geometry_msgs::msg::Pose getMostLikelyPose(
        const nav_msgs::msg::Odometry& prev_odom,
        const nav_msgs::msg::Odometry& curr_odom,
        const geometry_msgs::msg::Pose& prev_pose) = 0;

    virtual void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) = 0;

    virtual void activate() = 0;

    virtual void deactivate() = 0;

    virtual void cleanup() = 0;

protected:
    // Noise parameters
    double alpha1_;
    double alpha2_;
    double alpha3_;
    double alpha4_;
    double alpha5_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};  
} // nav2_localization

#endif // NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_BASE_HPP_
