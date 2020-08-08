#ifndef NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_BASE_HPP_
#define NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_BASE_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nav2_localization_base
{

/**
 * @class SampleMotionModel
 * @brief Abstract interface for sample motion model to adhere to with pluginlib
 */
class SampleMotionModel
{   
public:
    using Ptr = std::shared_ptr<nav2_localization_base::SampleMotionModel>;

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

protected:
    SampleMotionModel(){}
};  
} // nav2_localization_base

#endif // NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_BASE_HPP_
