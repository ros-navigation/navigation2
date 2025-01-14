// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/critic_manager.hpp"

// Tests critic manager

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace mppi;  // NOLINT
using namespace mppi::critics;  // NOLINT

class DummyCritic : public CriticFunction
{
public:
  virtual void initialize() {initialized_ = true;}
  virtual void score(CriticData & /*data*/) {scored_ = true;}
  bool initialized_{false}, scored_{false};
};

class CriticManagerWrapper : public CriticManager
{
public:
  CriticManagerWrapper()
  : CriticManager() {}

  virtual ~CriticManagerWrapper() = default;

  virtual void loadCritics()
  {
    critics_.clear();
    auto instance = std::unique_ptr<critics::CriticFunction>(new DummyCritic);
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + "DummyCritic", costmap_ros_,
      parameters_handler_);
  }

  std::string getFullNameWrapper(const std::string & name)
  {
    return getFullName(name);
  }

  bool getDummyCriticInitialized()
  {
    const auto critic = critics_[0].get();
    if (critic == nullptr) {
      return false;
    }
    const auto dummy_critic = dynamic_cast<DummyCritic *>(critic);
    return dummy_critic == nullptr ? false : dummy_critic->initialized_;
  }

  bool getDummyCriticScored()
  {
    const auto critic = critics_[0].get();
    if (critic == nullptr) {
      return false;
    }
    const auto dummy_critic = dynamic_cast<DummyCritic *>(critic);
    return dummy_critic == nullptr ? false : dummy_critic->scored_;
  }
};

class CriticManagerWrapperEnum : public CriticManager
{
public:
  CriticManagerWrapperEnum()
  : CriticManager() {}

  virtual ~CriticManagerWrapperEnum() = default;

  unsigned int getCriticNum()
  {
    return critics_.size();
  }
};

TEST(CriticManagerTests, BasicCriticOperations)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  // Configuration should get parameters and initialize critic functions
  CriticManagerWrapper critic_manager;
  critic_manager.on_configure(node, "critic_manager", costmap_ros, &param_handler);
  EXPECT_TRUE(critic_manager.getDummyCriticInitialized());

  // Evaluation of critics should score them, but only if failure flag is not set
  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  models::Path path;
  geometry_msgs::msg::Pose goal;
  xt::xtensor<float, 1> costs;
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};

  data.fail_flag = true;
  EXPECT_FALSE(critic_manager.getDummyCriticScored());
  data.fail_flag = false;
  critic_manager.evalTrajectoriesScores(data);
  EXPECT_TRUE(critic_manager.getDummyCriticScored());

  // This should get the full namespaced name of the critics
  EXPECT_EQ(critic_manager.getFullNameWrapper("name"), std::string("mppi::critics::name"));
}

TEST(CriticManagerTests, CriticLoadingTest)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  node->declare_parameter(
    "critic_manager.critics",
    rclcpp::ParameterValue(std::vector<std::string>{"ConstraintCritic", "PreferForwardCritic"}));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State state;
  costmap_ros->on_configure(state);

  // This should grab the critics parameter and load the 2 requested plugins
  CriticManagerWrapperEnum critic_manager;
  critic_manager.on_configure(node, "critic_manager", costmap_ros, &param_handler);
  EXPECT_EQ(critic_manager.getCriticNum(), 2u);
}
