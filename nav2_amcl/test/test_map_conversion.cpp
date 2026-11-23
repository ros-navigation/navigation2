// Copyright (c) 2026 Open Navigation LLC
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

#include "gtest/gtest.h"
#include "nav2_amcl/amcl_node.hpp"
#include "nav2_amcl/map/map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/utilities.hpp"

class AmclMapConversionTester : public nav2_amcl::AmclNode
{
public:
  using nav2_amcl::AmclNode::AmclNode;
  using nav2_amcl::AmclNode::convertMap;
};

// The OccupancyGrid origin is the corner of cell (0, 0), so the center of cell i
// is at origin + (i + 0.5) * resolution, matching Costmap2D::mapToWorld.
TEST(AmclMapConversion, origin_is_cell_corner)
{
  auto amcl = std::make_shared<AmclMapConversionTester>(rclcpp::NodeOptions());

  for (const unsigned int size : {10u, 11u}) {
    nav_msgs::msg::OccupancyGrid msg;
    msg.info.width = size;
    msg.info.height = size;
    msg.info.resolution = 0.05;
    msg.info.origin.position.x = -1.3;
    msg.info.origin.position.y = 2.7;
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(size * size, 0);

    map_t * map = amcl->convertMap(msg);
    const double res = msg.info.resolution;
    const double ox = msg.info.origin.position.x;
    const double oy = msg.info.origin.position.y;

    for (int i = 0; i < static_cast<int>(size); ++i) {
      EXPECT_NEAR(MAP_WXGX(map, i), ox + (i + 0.5) * res, 1e-9);
      EXPECT_NEAR(MAP_WYGY(map, i), oy + (i + 0.5) * res, 1e-9);

      // Points anywhere inside the cell look up that cell.
      EXPECT_EQ(MAP_GXWX(map, ox + (i + 0.1) * res), i);
      EXPECT_EQ(MAP_GXWX(map, ox + (i + 0.9) * res), i);
      EXPECT_EQ(MAP_GYWY(map, oy + (i + 0.1) * res), i);
      EXPECT_EQ(MAP_GYWY(map, oy + (i + 0.9) * res), i);
    }

    map_free(map);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
