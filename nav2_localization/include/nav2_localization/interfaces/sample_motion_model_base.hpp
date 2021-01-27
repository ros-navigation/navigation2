// Copyright (c) 2021 Khaled SAAD and Jose M. TORRES-CAMARA

#ifndef NAV2_LOCALIZATION__INTERFACES__SAMPLE_MOTION_MODEL_BASE_HPP_
#define NAV2_LOCALIZATION__INTERFACES__SAMPLE_MOTION_MODEL_BASE_HPP_

#include <memory>  // For shared_ptr<>
#include "geometry_msgs/msg/transform_stamped.hpp"
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
    using Ptr = std::shared_ptr<nav2_localization::SampleMotionModel>;

    /**
     * @brief Calculates the most likely pose that the robot is now in, following its motion.
     * @param prev_odom The robot's odometry at the previous time step (i.e. before the robot has moved).
     * @param curr_odom The robot's odometry at the current time step (i.e. after the robot has moved).
     * @param prev_pose The robot's pose estimation at the previous time step.
     * @return The most likely pose of the robot at the current time step, based on the model's estimation.
     */
    virtual geometry_msgs::msg::TransformStamped getMostLikelyPose(
        const geometry_msgs::msg::TransformStamped& prev_odom,
        const geometry_msgs::msg::TransformStamped& curr_odom,
        const geometry_msgs::msg::TransformStamped& prev_pose) = 0;

    /**
     * @brief Configures the model, during the "Configuring" state of the parent lifecycle node.
     * @param node Pointer to the parent lifecycle node.
     */     
    virtual void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) = 0;

    /**
     * @brief Activates the model, during the "Activating" state of the parent lifecycle node.
     */
    virtual void activate() = 0;

    /**
     * @brief Deactivates the model, during the "Decativating" state of the parent lifecycle node. 
     */
    virtual void deactivate() = 0;

    /**
     * @brief Cleans up the model, during the "Cleaningup" state of the parent lifecycle node.
     */
    virtual void cleanup() = 0;

protected:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__INTERFACES__SAMPLE_MOTION_MODEL_BASE_HPP_
