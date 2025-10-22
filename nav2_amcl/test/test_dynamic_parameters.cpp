// Copyright (c) 2025 Maurice Alexander Purnawan
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

#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_amcl/amcl_node.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(WPTest, test_dynamic_parameters)
{
  auto amcl = std::make_shared<nav2_amcl::AmclNode>();
  amcl->configure();
  amcl->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    amcl->get_node_base_interface(), amcl->get_node_topics_interface(),
    amcl->get_node_graph_interface(),
    amcl->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("alpha1", 1.0),
      rclcpp::Parameter("alpha2", 2.0),
      rclcpp::Parameter("alpha3", 3.0),
      rclcpp::Parameter("alpha4", 4.0),
      rclcpp::Parameter("alpha5", 5.0),
      rclcpp::Parameter("beam_skip_distance", 0.5),
      rclcpp::Parameter("beam_skip_threshold", 0.2),
      rclcpp::Parameter("lambda_short", 0.1),
      rclcpp::Parameter("laser_likelihood_max_dist", 0.3),
      rclcpp::Parameter("laser_min_range", 0.05),
      rclcpp::Parameter("laser_max_range", 10.0),
      rclcpp::Parameter("z_hit", 0.95),
      rclcpp::Parameter("z_max", 0.95),
      rclcpp::Parameter("z_rand", 0.95),
      rclcpp::Parameter("z_short", 0.95),
      rclcpp::Parameter("pf_err", 0.02),
      rclcpp::Parameter("pf_z", 0.1),
      rclcpp::Parameter("recovery_alpha_fast", 0.1),
      rclcpp::Parameter("recovery_alpha_slow", 0.001),
      rclcpp::Parameter("save_pose_rate", 2.0),
      rclcpp::Parameter("sigma_hit", 0.1),
      rclcpp::Parameter("transform_tolerance", 0.3),
      rclcpp::Parameter("update_min_a", 0.05),
      rclcpp::Parameter("update_min_d", 0.1),
      rclcpp::Parameter("max_beams", 60),
      rclcpp::Parameter("max_particles", 1000),
      rclcpp::Parameter("min_particles", 100),
      rclcpp::Parameter("resample_interval", 1),
      rclcpp::Parameter("base_frame_id", "base_footprint"),
      rclcpp::Parameter("global_frame_id", "map"),
      rclcpp::Parameter("map_topic", "map"),
      rclcpp::Parameter("laser_model_type", "likelihood_field"),
      rclcpp::Parameter("odom_frame_id", "odom"),
      rclcpp::Parameter("scan_topic", "scan"),
      rclcpp::Parameter("robot_model_type", "nav2_amcl::OmniMotionModel"),
      rclcpp::Parameter("tf_broadcast", true),
      rclcpp::Parameter("do_beamskip", false),
      rclcpp::Parameter("set_initial_pose", false),
      rclcpp::Parameter("first_map_only", false)});

  rclcpp::spin_until_future_complete(
    amcl->get_node_base_interface(),
    results);

  EXPECT_EQ(amcl->get_parameter("alpha1").as_double(), 1.0);
  EXPECT_EQ(amcl->get_parameter("alpha2").as_double(), 2.0);
  EXPECT_EQ(amcl->get_parameter("alpha3").as_double(), 3.0);
  EXPECT_EQ(amcl->get_parameter("alpha4").as_double(), 4.0);
  EXPECT_EQ(amcl->get_parameter("alpha5").as_double(), 5.0);
  EXPECT_EQ(amcl->get_parameter("beam_skip_distance").as_double(), 0.5);
  EXPECT_EQ(amcl->get_parameter("beam_skip_threshold").as_double(), 0.2);
  EXPECT_EQ(amcl->get_parameter("lambda_short").as_double(), 0.1);
  EXPECT_EQ(amcl->get_parameter("laser_likelihood_max_dist").as_double(), 0.3);
  EXPECT_EQ(amcl->get_parameter("laser_min_range").as_double(), 0.05);
  EXPECT_EQ(amcl->get_parameter("laser_max_range").as_double(), 10.0);
  EXPECT_EQ(amcl->get_parameter("z_hit").as_double(), 0.95);
  EXPECT_EQ(amcl->get_parameter("z_max").as_double(), 0.95);
  EXPECT_EQ(amcl->get_parameter("z_rand").as_double(), 0.95);
  EXPECT_EQ(amcl->get_parameter("z_short").as_double(), 0.95);
  EXPECT_EQ(amcl->get_parameter("pf_err").as_double(), 0.02);
  EXPECT_EQ(amcl->get_parameter("pf_z").as_double(), 0.1);
  EXPECT_EQ(amcl->get_parameter("recovery_alpha_fast").as_double(), 0.1);
  EXPECT_EQ(amcl->get_parameter("recovery_alpha_slow").as_double(), 0.001);
  EXPECT_EQ(amcl->get_parameter("save_pose_rate").as_double(), 2.0);
  EXPECT_EQ(amcl->get_parameter("sigma_hit").as_double(), 0.1);
  EXPECT_EQ(amcl->get_parameter("transform_tolerance").as_double(), 0.3);
  EXPECT_EQ(amcl->get_parameter("update_min_a").as_double(), 0.05);
  EXPECT_EQ(amcl->get_parameter("update_min_d").as_double(), 0.1);
  EXPECT_EQ(amcl->get_parameter("max_beams").as_int(), 60);
  EXPECT_EQ(amcl->get_parameter("max_particles").as_int(), 1000);
  EXPECT_EQ(amcl->get_parameter("min_particles").as_int(), 100);
  EXPECT_EQ(amcl->get_parameter("resample_interval").as_int(), 1);
  EXPECT_EQ(amcl->get_parameter("base_frame_id").as_string(), "base_footprint");
  EXPECT_EQ(amcl->get_parameter("global_frame_id").as_string(), "map");
  EXPECT_EQ(amcl->get_parameter("map_topic").as_string(), "map");
  EXPECT_EQ(amcl->get_parameter("laser_model_type").as_string(), "likelihood_field");
  EXPECT_EQ(amcl->get_parameter("odom_frame_id").as_string(), "odom");
  EXPECT_EQ(amcl->get_parameter("scan_topic").as_string(), "scan");
  EXPECT_EQ(amcl->get_parameter("robot_model_type").as_string(), "nav2_amcl::OmniMotionModel");
  EXPECT_EQ(amcl->get_parameter("tf_broadcast").as_bool(), true);
  EXPECT_EQ(amcl->get_parameter("do_beamskip").as_bool(), false);
  EXPECT_EQ(amcl->get_parameter("set_initial_pose").as_bool(), false);
  EXPECT_EQ(amcl->get_parameter("first_map_only").as_bool(), false);

  results = rec_param->set_parameters_atomically({rclcpp::Parameter("alpha1", -1.0)});
  rclcpp::spin_until_future_complete(amcl->get_node_base_interface(), results);
  EXPECT_EQ(amcl->get_parameter("alpha1").as_double(), 1.0);

  results = rec_param->set_parameters_atomically({rclcpp::Parameter("beam_skip_distance", -0.5)});
  rclcpp::spin_until_future_complete(amcl->get_node_base_interface(), results);
  EXPECT_EQ(amcl->get_parameter("beam_skip_distance").as_double(), 0.5);

  results = rec_param->set_parameters_atomically({rclcpp::Parameter("resample_interval", 0)});
  rclcpp::spin_until_future_complete(amcl->get_node_base_interface(), results);
  EXPECT_EQ(amcl->get_parameter("resample_interval").as_int(), 1);

  results = rec_param->set_parameters_atomically({rclcpp::Parameter("max_particles", 50)});
  rclcpp::spin_until_future_complete(amcl->get_node_base_interface(), results);
  EXPECT_EQ(amcl->get_parameter("max_particles").as_int(), 1000);

  results = rec_param->set_parameters_atomically({rclcpp::Parameter("min_particles", 2000)});
  rclcpp::spin_until_future_complete(amcl->get_node_base_interface(), results);
  EXPECT_EQ(amcl->get_parameter("min_particles").as_int(), 100);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
