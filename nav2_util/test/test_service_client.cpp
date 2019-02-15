// Copyright (c) 2019 Intel Corporation
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

#include <string>
#include "nav2_util/service_client.hpp"
#include "std_srvs/srv/empty.hpp"
#include "gtest/gtest.h"

using nav2_util::ServiceClient;
using std::string;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestServiceClient : public ServiceClient<std_srvs::srv::Empty>
{
public:
  TestServiceClient(const std::string & name)
  : ServiceClient(name) {}

  string name() {return node_->get_name();}
};

TEST(ServiceClient, service_name_with_slash)
{
  TestServiceClient t("/foo/bar");
  string adjustedPrefix = "_foo_bar_Node_";
  ASSERT_EQ(t.name().length(), adjustedPrefix.length() + 8);
  ASSERT_EQ(0, t.name().compare(0, adjustedPrefix.length(), adjustedPrefix));
}
