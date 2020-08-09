#ifndef NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_PLUGINS_HPP_
#define NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_PLUGINS_HPP_

#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nav2_localization_plugins
{
class DummyMotionSampler : public nav2_localization_base::SampleMotionModel
{
public:
    DummyMotionSampler(){}

    geometry_msgs::msg::Pose getMostLikelyPose(
        const nav_msgs::msg::Odometry& prev_odom,
        const nav_msgs::msg::Odometry& curr_odom,
        const geometry_msgs::msg::Pose& prev_pose);

    void configure(
        const double& alpha1,
        const double& alpha2,
        const double& alpha3,
        const double& alpha4,
        const double& alpha5);
};
// // Implements a Bicycle motion model
// class Bicycle : public nav2_localization_base::SampleMotionModel
// {   
//     public:
//         Bicycle(){}
//         // TODO - How to send odometry?
//         std::vector<float> get_transformation(const Odometry& odom); // Get the position and orientation transformation using a given odometry
// };

// // Implements an Ackermann motion model
// //// TODO - Use this to simplify ackermann vehicle to bicycle model and then use the Bicycle model
// class Ackermann : public nav2_localization_base::MotionModel
// {   
//     public:
//         Ackermann(){}
//         // TODO - How to send odometry?
//         std::vector<float> get_transformation(const Odometry& odom); // Get the position and orientation transformation using a given odometry
// };

// // Implements a Differential motion model
// class Differential : public nav2_localization_base::MotionModel
// {   
//     public:
//         Differential(){}
//         // TODO - How to send odometry?
//         std::vector<float> get_transformation(const Odometry& odom); // Get the position and orientation transformation using a given odometry
// };  

// // Implements an Omnidirectional motion model
// class Omnidirectional : public nav2_localization_base::MotionModel
// {   
//     public:
//         Omnidirectional(){}
//         // TODO - How to send odometry?
//         std::vector<float> get_transformation(const Odometry& odom); // Get the position and orientation transformation using a given odometry
// };  
}

#endif // NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_PLUGINS_HPP_
