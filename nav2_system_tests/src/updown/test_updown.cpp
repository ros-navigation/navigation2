// Copyright (c) 2018 Intel Corporation
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

#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "rcutils/cmdline_parser.h"

using namespace std::chrono_literals;

struct xytheta
{
  double x;
  double y;
  double theta;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  nav2_lifecycle_manager::LifecycleManagerClient client;

  // Create a set of target poses across the map
  std::vector<xytheta> target_poses;
  target_poses.push_back({-2.0, -0.5, 0});
  target_poses.push_back({0.94, -0.55, 0});
  target_poses.push_back({1.7, 0.5, 1.4});
  target_poses.push_back({0.97, 1.68, 2.94});
  target_poses.push_back({0.02, 1.74, -2.9});

  xytheta & initial_pose = target_poses[0];

  // Wait for a few seconds to let all of the nodes come up
  std::this_thread::sleep_for(5s);

  // Start the nav2 system, bringing it to the ACTIVE state
  client.startup();

  // Set the robot's starting pose (approximately where it comes up in gazebo)
  client.set_initial_pose(initial_pose.x, initial_pose.y, initial_pose.theta);

  // Wait for a couple secs to make sure the nodes have processed all discovery
  // info before starting
  std::this_thread::sleep_for(2s);

  // Parse the command line options
  char * nav_type_arg = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nav_type_arg != nullptr) {
    std::string nav_type(nav_type_arg);

    if (nav_type == "iterative") {
      // In the iterative case, navigate through all of the poses (but skip the
      // first one, which is the initial pose)
      for (std::vector<xytheta>::size_type i = 1; i < target_poses.size(); i++) {
        auto pose = target_poses[i];
        if (!client.navigate_to_pose(pose.x, pose.y, pose.theta)) {
          RCLCPP_ERROR(rclcpp::get_logger("test_updown"), "Navigation failed!");
          break;
        }
      }
    } else if (nav_type == "random") {
      // In the random case, navigate to randomly-selected poses from the target_poses
      // collection

      // Get set up to generate random indices
      std::random_device r;
      std::default_random_engine e1(r());
      std::uniform_int_distribution<int> uniform_dist(0, target_poses.size() - 1);

      for (int i = 0, cur_index = 0; i < 10; i++) {
        // Get a random index that is not the current one (so we can navigate
        // to a pose different than our current location)
        int next_index = 0;
        do {
          next_index = uniform_dist(r);
        } while (next_index == cur_index);

        // Grab the pose for that index and start the navigation
        auto pose = target_poses[next_index];
        if (!client.navigate_to_pose(pose.x, pose.y, pose.theta)) {
          RCLCPP_ERROR(rclcpp::get_logger("test_updown"), "Navigation failed!");
          break;
        }
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("test_updown"),
        "Unrecognized test type: %s, running simple up/down test\n", nav_type);
    }
  }

  // Shut down the nav2 system, bringing it to the FINALIZED state
  client.shutdown();

  rclcpp::shutdown();
  return 0;
}
