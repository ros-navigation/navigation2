// Copyright (c) 2020, Samsung Research America
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
#include "nav2_smac_planner/utils.hpp"

using namespace nav2_smac_planner;

TEST(transform_footprint_to_edges, test_basic)
{
    geometry_msgs::msg::Point p1;
    p1.x = 1.0;
    p1.y = 1.0;

    geometry_msgs::msg::Point p2;
    p2.x = 1.0;
    p2.y = -1.0;

    geometry_msgs::msg::Point p3;
    p3.x = -1.0;
    p3.y = -1.0;

    geometry_msgs::msg::Point p4;
    p4.x = -1.0;
    p4.y = 1.0;

    std::vector<geometry_msgs::msg::Point> footprint{p1, p2, p3, p4};

    auto result = transformFootprintToEdges(0.0, 0.0, 0.0, footprint);

    EXPECT_EQ(result.size(), 8u);
}