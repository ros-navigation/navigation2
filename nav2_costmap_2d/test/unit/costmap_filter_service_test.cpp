// Copyright (c) 2022 Samsung R&D Institute Russia
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

#include <gtest/gtest.h>

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"
#include "std_srvs/srv/set_bool.hpp"

static const char FILTER_NAME[]{"costmap_filter"};

class CostmapFilterWrapper : public nav2_costmap_2d::CostmapFilter
{
public:
  // Dummy implementations of virtual methods
  void initializeFilter(
    const std::string &) {}

  void process(
    nav2_costmap_2d::Costmap2D &,
    int, int, int, int,
    const geometry_msgs::msg::Pose2D &) {}

  void resetFilter() {}

  // Actual testing methods
  void setName(const std::string & name)
  {
    name_ = name;
  }

  void setNode(const nav2_util::LifecycleNode::WeakPtr & node)
  {
    node_ = node;
  }

  bool getEnabled()
  {
    return enabled_;
  }
};

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    // Create new LifecycleNode
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_node");

    // Create new CostmapFilter
    costmap_filter_ = std::make_shared<CostmapFilterWrapper>();
    costmap_filter_->setNode(node_);
    costmap_filter_->setName(FILTER_NAME);

    // Set CostmapFilter ROS-parameters
    node_->declare_parameter(
      std::string(FILTER_NAME) + ".filter_info_topic", rclcpp::ParameterValue("filter_info"));
    node_->set_parameter(
      rclcpp::Parameter(std::string(FILTER_NAME) + ".filter_info_topic", "filter_info"));
  }

  ~TestNode()
  {
    costmap_filter_.reset();
    node_.reset();
  }

  template<class T>
  typename T::Response::SharedPtr send_request(
    nav2_util::LifecycleNode::SharedPtr node,
    typename rclcpp::Client<T>::SharedPtr client,
    typename T::Request::SharedPtr request)
  {
    auto result = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      return result.get();
    } else {
      return nullptr;
    }
  }

protected:
  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<CostmapFilterWrapper> costmap_filter_;
};

TEST_F(TestNode, testEnableService)
{
  costmap_filter_->onInitialize();

  RCLCPP_INFO(node_->get_logger(), "Testing enabling service");
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto client = node_->create_client<std_srvs::srv::SetBool>(
    std::string(FILTER_NAME) + "/toggle_filter");

  RCLCPP_INFO(node_->get_logger(), "Waiting for enabling service");
  ASSERT_TRUE(client->wait_for_service());

  // Set costmap filter enabled
  req->data = true;
  auto resp = send_request<std_srvs::srv::SetBool>(node_, client, req);

  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);
  ASSERT_EQ(resp->message, "Enabled");
  ASSERT_TRUE(costmap_filter_->getEnabled());

  // Set costmap filter disabled
  req->data = false;
  resp = send_request<std_srvs::srv::SetBool>(node_, client, req);

  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);
  ASSERT_EQ(resp->message, "Disabled");
  ASSERT_FALSE(costmap_filter_->getEnabled());
}

int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
  rclcpp::shutdown();

  return test_result;
}
