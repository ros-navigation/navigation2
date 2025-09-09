// Copyright (c) 2025, Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <memory>
#include <string>

#include "nav2_route/plugins/route_operations/time_marker.hpp"

namespace nav2_route
{

void TimeMarker::configure(
  const nav2_util::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/* costmap_subscriber */,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring Adjust speed limit operation.");
  name_ = name;
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".time_tag", rclcpp::ParameterValue("abs_time_taken"));
  time_tag_ = node->get_parameter(getName() + ".time_tag").as_string();
  clock_ = node->get_clock();
  edge_start_time_ = rclcpp::Time(0.0);
}

OperationResult TimeMarker::perform(
  NodePtr /*node_achieved*/,
  EdgePtr edge_entered,
  EdgePtr edge_exited,
  const Route & /*route*/,
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
  const Metadata * /*mdata*/)
{
  OperationResult result;
  rclcpp::Time now = clock_->now();
  if (!edge_exited || edge_exited->edgeid != curr_edge_) {
    edge_start_time_ = now;
    curr_edge_ = edge_entered ? edge_entered->edgeid : 0;
    return result;
  }

  float dur = static_cast<float>((now - edge_start_time_).seconds());
  if (edge_start_time_.seconds() > 1e-3) {
    edge_exited->metadata.setValue<float>(time_tag_, dur);
  }

  edge_start_time_ = now;
  curr_edge_ = edge_entered ? edge_entered->edgeid : 0;
  return result;
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::TimeMarker, nav2_route::RouteOperation)
