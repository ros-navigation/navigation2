// Copyright (c) 2025 <Your Name>
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef NAV2_COLLISION_MONITOR__COSTMAP_HPP_
#define NAV2_COLLISION_MONITOR__COSTMAP_HPP_

#include <memory>                   // shared_ptr / weak_ptr
#include <string>                   // std::string
#include <vector>                   // std::vector
#include "nav2_collision_monitor/source.hpp"        // Base class Source (CM’s plugin interface)
#include "nav2_msgs/msg/costmap.hpp"                // CHANGE: use Nav2 Costmap message
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav2_ros_common/node_utils.hpp>

namespace nav2_collision_monitor
{

class CostmapSource : public Source                  // Inherit CM’s Source (same as other inputs)
{
public:
  CostmapSource(                                     // Constructor declaration (no return type)
    const nav2::LifecycleNode::WeakPtr & node ,  // Non-owning pointer to lifecycle node
    const std::string & source_name,                 // Name used in params (e.g., "costmap")
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,// Shared TF buffer
    const std::string & base_frame_id,               // Usually "base_link"
    const std::string & global_frame_id,             // Usually "odom" or "map"
    const tf2::Duration & transform_tolerance,       // TF timing slack
    const rclcpp::Duration & source_timeout,         // Max data age allowed
    const bool base_shift_correction);               // Whether to correct base motion drift

  ~CostmapSource();                                  // Virtual dtor not required; explicit is fine

  void configure();                         // Where we declare params & create subscription

  bool getData(                                      // CM calls this each tick to fetch obstacle points
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) override;

  void getParameters(std::string & source_topic);    // Read: <source_name>.topic, <source_name>.cost_threshold

private:
  void dataCallback(nav2_msgs::msg::Costmap::ConstSharedPtr msg);
  // ↑ Store the latest Costmap message; we’ll read it in getData()

  nav2_msgs::msg::Costmap::ConstSharedPtr data_;     // Latest costmap message (thread-safe shared ptr)
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr data_sub_;
  // ↑ The actual ROS2 subscription

  int cost_threshold_{254};                          // Default: lethal cells only (254). Make it a param.
  // (Nav2 costs: 0 free, 253 inscribed, 254 lethal, 255 unknown)

  
};

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__COSTMAP_HPP_
