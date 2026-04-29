// Copyright (c) 2021 RoboTech Vision
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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <chrono>
#include <thread>

#include "nav2_core/smoother.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "tf2_ros/buffer.hpp"

using namespace std::chrono_literals;

// A smoother for testing the base class

class DummySmoother : public nav2_core::Smoother
{
public:
  DummySmoother() {}

  ~DummySmoother() {}

  virtual void configure(
    const nav2::LifecycleNode::WeakPtr &,
    std::string, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) {}

  virtual void cleanup() {}

  virtual void activate() {}

  virtual void deactivate() {}

  virtual bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time)
  {
    assert(path.poses.size() == 2);

    if (path.poses.front() == path.poses.back()) {
      throw nav2_core::PlannerException("Start and goal pose must differ");
    }

    auto max_time_ms = max_time.to_chrono<std::chrono::milliseconds>();
    std::this_thread::sleep_for(std::min(max_time_ms, 100ms));

    // place dummy pose in the middle of the path
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x =
      (path.poses.front().pose.position.x + path.poses.back().pose.position.x) / 2;
    pose.pose.position.y =
      (path.poses.front().pose.position.y + path.poses.back().pose.position.y) / 2;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);

    return max_time_ms > 100ms;
  }

private:
  std::string command_;
  std::chrono::system_clock::time_point start_time_;
};

PLUGINLIB_EXPORT_CLASS(DummySmoother, nav2_core::Smoother)
