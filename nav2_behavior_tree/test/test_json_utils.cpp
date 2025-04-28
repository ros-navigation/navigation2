// Copyright (c) 2025 Alberto J. Tudela Roldán
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

#include <gtest/gtest.h>

#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "nav2_msgs/msg/waypoint_status.hpp"


class JsonTest : public testing::Test
{
protected:
  JsonTest()
  {
    BT::JsonExporter & exporter = BT::JsonExporter::get();
    exporter.addConverter<std::chrono::milliseconds>();
    exporter.addConverter<builtin_interfaces::msg::Time>();
    exporter.addConverter<std_msgs::msg::Header>();
    exporter.addConverter<geometry_msgs::msg::Point>();
    exporter.addConverter<geometry_msgs::msg::Quaternion>();
    exporter.addConverter<geometry_msgs::msg::Pose>();
    exporter.addConverter<geometry_msgs::msg::PoseStamped>();
    exporter.addConverter<std::vector<geometry_msgs::msg::PoseStamped>>();
    exporter.addConverter<nav_msgs::msg::Goals>();
    exporter.addConverter<nav_msgs::msg::Path>();
    exporter.addConverter<nav2_msgs::msg::WaypointStatus>();
    exporter.addConverter<std::vector<nav2_msgs::msg::WaypointStatus>>();
  }
};

TEST_F(JsonTest, test_milliseconds)
{
  BT::JsonExporter & exporter = BT::JsonExporter::get();

  std::chrono::milliseconds milliseconds_test(1000);

  nlohmann::json json;
  exporter.toJson(BT::Any(milliseconds_test), json["ms"]);

  std::cout << json.dump(2) << std::endl;

  ASSERT_EQ(json["ms"]["__type"], "std::chrono::milliseconds");
  ASSERT_EQ(json["ms"]["ms"], 1000);

  // Check the two-ways transform, i.e. "from_json"
  auto milliseconds_test2 =
    exporter.fromJson(json["ms"])->first.cast<std::chrono::milliseconds>();

  ASSERT_EQ(milliseconds_test.count(), milliseconds_test2.count());

  // Convert from string
  std::chrono::milliseconds milliseconds_test3;
  auto const test_json = R"(json:{"__type": "std::chrono::milliseconds", "ms": 1000})";
  ASSERT_NO_THROW(milliseconds_test3 = BT::convertFromString<std::chrono::milliseconds>(test_json));

  ASSERT_EQ(milliseconds_test.count(), milliseconds_test3.count());
}

TEST_F(JsonTest, test_time)
{
  BT::JsonExporter & exporter = BT::JsonExporter::get();

  builtin_interfaces::msg::Time time_test;
  time_test.sec = 1;
  time_test.nanosec = 2;

  nlohmann::json json;
  exporter.toJson(BT::Any(time_test), json["time"]);

  std::cout << json.dump(2) << std::endl;

  ASSERT_EQ(json["time"]["__type"], "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["time"]["sec"], 1);
  ASSERT_EQ(json["time"]["nanosec"], 2);

  // Check the two-ways transform, i.e. "from_json"
  auto time_test2 = exporter.fromJson(json["time"])->first.cast<builtin_interfaces::msg::Time>();

  ASSERT_EQ(time_test.sec, time_test2.sec);
  ASSERT_EQ(time_test.nanosec, time_test2.nanosec);

  // Convert from string
  builtin_interfaces::msg::Time time_test3;
  auto const test_json =
    R"(json:{"__type": "builtin_interfaces::msg::Time", "sec": 1, "nanosec": 2})";
  ASSERT_NO_THROW(time_test3 = BT::convertFromString<builtin_interfaces::msg::Time>(test_json));

  ASSERT_EQ(time_test.sec, time_test3.sec);
  ASSERT_EQ(time_test.nanosec, time_test3.nanosec);
}

TEST_F(JsonTest, test_header)
{
  BT::JsonExporter & exporter = BT::JsonExporter::get();

  std_msgs::msg::Header header_test;
  header_test.stamp.sec = 1;
  header_test.stamp.nanosec = 2;
  header_test.frame_id = "map";

  nlohmann::json json;
  exporter.toJson(BT::Any(header_test), json["header"]);

  std::cout << json.dump(2) << std::endl;

  ASSERT_EQ(json["header"]["__type"], "std_msgs::msg::Header");
  ASSERT_EQ(json["header"]["stamp"]["__type"], "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["header"]["stamp"]["sec"], 1);
  ASSERT_EQ(json["header"]["stamp"]["nanosec"], 2);
  ASSERT_EQ(json["header"]["frame_id"], "map");

  // Check the two-ways transform, i.e. "from_json"
  auto header_test2 = exporter.fromJson(json["header"])->first.cast<std_msgs::msg::Header>();

  ASSERT_EQ(header_test.stamp.sec, header_test2.stamp.sec);
  ASSERT_EQ(header_test.stamp.nanosec, header_test2.stamp.nanosec);
  ASSERT_EQ(header_test.frame_id, header_test2.frame_id);

  // Convert from string
  std_msgs::msg::Header header_test3;
  auto const test_json =
    R"(json:
      {
        "__type": "std_msgs::msg::Header",
        "stamp": {"__type": "builtin_interfaces::msg::Time", "sec": 1, "nanosec": 2},
        "frame_id": "map"
      }
    )";
  ASSERT_NO_THROW(header_test3 = BT::convertFromString<std_msgs::msg::Header>(test_json));
  ASSERT_EQ(header_test.stamp.sec, header_test3.stamp.sec);
  ASSERT_EQ(header_test.stamp.nanosec, header_test3.stamp.nanosec);
  ASSERT_EQ(header_test.frame_id, header_test3.frame_id);
}

TEST_F(JsonTest, test_point)
{
  BT::JsonExporter & exporter = BT::JsonExporter::get();

  geometry_msgs::msg::Point point_test;
  point_test.x = 1.0;
  point_test.y = 2.0;
  point_test.z = 3.0;

  nlohmann::json json;
  exporter.toJson(BT::Any(point_test), json["point"]);

  std::cout << json.dump(2) << std::endl;

  ASSERT_EQ(json["point"]["__type"], "geometry_msgs::msg::Point");
  ASSERT_EQ(json["point"]["x"], 1.0);
  ASSERT_EQ(json["point"]["y"], 2.0);
  ASSERT_EQ(json["point"]["z"], 3.0);

  // Check the two-ways transform, i.e. "from_json"
  auto point_test2 = exporter.fromJson(json["point"])->first.cast<geometry_msgs::msg::Point>();

  ASSERT_EQ(point_test.x, point_test2.x);
  ASSERT_EQ(point_test.y, point_test2.y);
  ASSERT_EQ(point_test.z, point_test2.z);

  // Convert from string
  geometry_msgs::msg::Point point_test3;
  auto const test_json =
    R"(json:
      {
        "__type": "geometry_msgs::msg::Point",
        "x": 1.0, "y": 2.0, "z": 3.0
      }
    )";
  ASSERT_NO_THROW(point_test3 = BT::convertFromString<geometry_msgs::msg::Point>(test_json));
  ASSERT_EQ(point_test.x, point_test3.x);
  ASSERT_EQ(point_test.y, point_test3.y);
  ASSERT_EQ(point_test.z, point_test3.z);
}

TEST_F(JsonTest, test_quaternion)
{
  BT::JsonExporter & exporter = BT::JsonExporter::get();

  geometry_msgs::msg::Quaternion quaternion_test;
  quaternion_test.x = 1.0;
  quaternion_test.y = 2.0;
  quaternion_test.z = 3.0;
  quaternion_test.w = 4.0;

  nlohmann::json json;
  exporter.toJson(BT::Any(quaternion_test), json["quaternion"]);

  std::cout << json.dump(2) << std::endl;

  ASSERT_EQ(json["quaternion"]["__type"], "geometry_msgs::msg::Quaternion");
  ASSERT_EQ(json["quaternion"]["x"], 1.0);
  ASSERT_EQ(json["quaternion"]["y"], 2.0);
  ASSERT_EQ(json["quaternion"]["z"], 3.0);
  ASSERT_EQ(json["quaternion"]["w"], 4.0);

  // Check the two-ways transform, i.e. "from_json"
  auto quaternion_test2 =
    exporter.fromJson(json["quaternion"])->first.cast<geometry_msgs::msg::Quaternion>();

  ASSERT_EQ(quaternion_test.x, quaternion_test2.x);
  ASSERT_EQ(quaternion_test.y, quaternion_test2.y);
  ASSERT_EQ(quaternion_test.z, quaternion_test2.z);
  ASSERT_EQ(quaternion_test.w, quaternion_test2.w);

  // Convert from string
  geometry_msgs::msg::Quaternion quaternion_test3;
  auto const test_json =
    R"(json:
      {
        "__type": "geometry_msgs::msg::Quaternion",
        "x": 1.0, "y": 2.0, "z": 3.0, "w": 4.0
      }
    )";
  ASSERT_NO_THROW(
    quaternion_test3 =
    BT::convertFromString<geometry_msgs::msg::Quaternion>(test_json));
  ASSERT_EQ(quaternion_test.x, quaternion_test3.x);
  ASSERT_EQ(quaternion_test.y, quaternion_test3.y);
  ASSERT_EQ(quaternion_test.z, quaternion_test3.z);
  ASSERT_EQ(quaternion_test.w, quaternion_test3.w);
}

TEST_F(JsonTest, test_pose_stamped)
{
  BT::JsonExporter & exporter = BT::JsonExporter::get();

  geometry_msgs::msg::PoseStamped pose_stamped_test;
  pose_stamped_test.header.stamp.sec = 1;
  pose_stamped_test.header.stamp.nanosec = 2;
  pose_stamped_test.header.frame_id = "map";
  pose_stamped_test.pose.position.x = 3.0;
  pose_stamped_test.pose.position.y = 4.0;
  pose_stamped_test.pose.position.z = 5.0;
  pose_stamped_test.pose.orientation.x = 6.0;
  pose_stamped_test.pose.orientation.y = 7.0;
  pose_stamped_test.pose.orientation.z = 8.0;
  pose_stamped_test.pose.orientation.w = 9.0;

  nlohmann::json json;
  exporter.toJson(BT::Any(pose_stamped_test), json["pose"]);

  std::cout << json.dump(2) << std::endl;

  ASSERT_EQ(json["pose"]["__type"], "geometry_msgs::msg::PoseStamped");
  ASSERT_EQ(json["pose"]["header"]["__type"], "std_msgs::msg::Header");
  ASSERT_EQ(json["pose"]["header"]["stamp"]["__type"], "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["pose"]["header"]["stamp"]["sec"], 1);
  ASSERT_EQ(json["pose"]["header"]["stamp"]["nanosec"], 2);
  ASSERT_EQ(json["pose"]["header"]["frame_id"], "map");
  ASSERT_EQ(json["pose"]["pose"]["__type"], "geometry_msgs::msg::Pose");
  ASSERT_EQ(json["pose"]["pose"]["position"]["__type"], "geometry_msgs::msg::Point");
  ASSERT_EQ(json["pose"]["pose"]["position"]["x"], 3.0);
  ASSERT_EQ(json["pose"]["pose"]["position"]["y"], 4.0);
  ASSERT_EQ(json["pose"]["pose"]["position"]["z"], 5.0);
  ASSERT_EQ(json["pose"]["pose"]["orientation"]["__type"], "geometry_msgs::msg::Quaternion");
  ASSERT_EQ(json["pose"]["pose"]["orientation"]["x"], 6.0);
  ASSERT_EQ(json["pose"]["pose"]["orientation"]["y"], 7.0);
  ASSERT_EQ(json["pose"]["pose"]["orientation"]["z"], 8.0);
  ASSERT_EQ(json["pose"]["pose"]["orientation"]["w"], 9.0);

  // Check the two-ways transform, i.e. "from_json"
  auto pose_stamped_test2 =
    exporter.fromJson(json["pose"])->first.cast<geometry_msgs::msg::PoseStamped>();

  ASSERT_EQ(pose_stamped_test.header.stamp.sec, pose_stamped_test2.header.stamp.sec);
  ASSERT_EQ(pose_stamped_test.header.stamp.nanosec, pose_stamped_test2.header.stamp.nanosec);
  ASSERT_EQ(pose_stamped_test.header.frame_id, pose_stamped_test2.header.frame_id);
  ASSERT_EQ(pose_stamped_test.pose.position.x, pose_stamped_test2.pose.position.x);
  ASSERT_EQ(pose_stamped_test.pose.position.y, pose_stamped_test2.pose.position.y);
  ASSERT_EQ(pose_stamped_test.pose.position.z, pose_stamped_test2.pose.position.z);
  ASSERT_EQ(pose_stamped_test.pose.orientation.x, pose_stamped_test2.pose.orientation.x);
  ASSERT_EQ(pose_stamped_test.pose.orientation.y, pose_stamped_test2.pose.orientation.y);
  ASSERT_EQ(pose_stamped_test.pose.orientation.z, pose_stamped_test2.pose.orientation.z);
  ASSERT_EQ(pose_stamped_test.pose.orientation.w, pose_stamped_test2.pose.orientation.w);

  // Convert from string
  geometry_msgs::msg::PoseStamped pose_stamped_test3;
  auto const test_json =
    R"(json:
      {
        "__type": "geometry_msgs::msg::PoseStamped",
        "header": {
          "__type": "std_msgs::msg::Header",
          "stamp": {"__type": "builtin_interfaces::msg::Time", "sec": 1, "nanosec": 2},
          "frame_id": "map"
        },
        "pose": {
          "__type": "geometry_msgs::msg::Pose",
          "position": {
            "__type": "geometry_msgs::msg::Point",
            "x": 3.0, "y": 4.0, "z": 5.0
          },
          "orientation": {
            "__type": "geometry_msgs::msg::Quaternion",
            "x": 6.0, "y": 7.0, "z": 8.0, "w": 9.0
          }
        }
      }
    )";
  ASSERT_NO_THROW(
    pose_stamped_test3 =
    BT::convertFromString<geometry_msgs::msg::PoseStamped>(test_json));
  ASSERT_EQ(pose_stamped_test.header, pose_stamped_test3.header);
  ASSERT_EQ(pose_stamped_test.pose.position, pose_stamped_test3.pose.position);
  ASSERT_EQ(pose_stamped_test.pose.orientation, pose_stamped_test3.pose.orientation);
}

TEST_F(JsonTest, test_pose_stamped_vector)
{
  BT::JsonExporter & exporter = BT::JsonExporter::get();

  std::vector<geometry_msgs::msg::PoseStamped> pose_stamped_vector_test;
  pose_stamped_vector_test.resize(2);
  pose_stamped_vector_test[0].header.stamp.sec = 1;
  pose_stamped_vector_test[0].header.stamp.nanosec = 2;
  pose_stamped_vector_test[0].header.frame_id = "map";
  pose_stamped_vector_test[0].pose.position.x = 3.0;
  pose_stamped_vector_test[0].pose.position.y = 4.0;
  pose_stamped_vector_test[0].pose.position.z = 5.0;
  pose_stamped_vector_test[0].pose.orientation.x = 6.0;
  pose_stamped_vector_test[0].pose.orientation.y = 7.0;
  pose_stamped_vector_test[0].pose.orientation.z = 8.0;
  pose_stamped_vector_test[0].pose.orientation.w = 9.0;

  pose_stamped_vector_test[1].header.stamp.sec = 10;
  pose_stamped_vector_test[1].header.stamp.nanosec = 11;
  pose_stamped_vector_test[1].header.frame_id = "odom";
  pose_stamped_vector_test[1].pose.position.x = 12.0;
  pose_stamped_vector_test[1].pose.position.y = 13.0;
  pose_stamped_vector_test[1].pose.position.z = 14.0;
  pose_stamped_vector_test[1].pose.orientation.x = 15.0;
  pose_stamped_vector_test[1].pose.orientation.y = 16.0;
  pose_stamped_vector_test[1].pose.orientation.z = 17.0;
  pose_stamped_vector_test[1].pose.orientation.w = 18.0;

  nlohmann::json json;
  exporter.toJson(BT::Any(pose_stamped_vector_test), json["poses"]);

  std::cout << json.dump(2) << std::endl;

  ASSERT_EQ(json["poses"][0]["__type"], "geometry_msgs::msg::PoseStamped");
  ASSERT_EQ(json["poses"][0]["header"]["__type"], "std_msgs::msg::Header");
  ASSERT_EQ(json["poses"][0]["header"]["stamp"]["__type"], "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["poses"][0]["header"]["stamp"]["sec"], 1);
  ASSERT_EQ(json["poses"][0]["header"]["stamp"]["nanosec"], 2);
  ASSERT_EQ(json["poses"][0]["header"]["frame_id"], "map");
  ASSERT_EQ(json["poses"][0]["pose"]["__type"], "geometry_msgs::msg::Pose");
  ASSERT_EQ(json["poses"][0]["pose"]["position"]["__type"], "geometry_msgs::msg::Point");
  ASSERT_EQ(json["poses"][0]["pose"]["position"]["x"], 3.0);
  ASSERT_EQ(json["poses"][0]["pose"]["position"]["y"], 4.0);
  ASSERT_EQ(json["poses"][0]["pose"]["position"]["z"], 5.0);
  ASSERT_EQ(
    json["poses"][0]["pose"]["orientation"]["__type"],
    "geometry_msgs::msg::Quaternion");
  ASSERT_EQ(json["poses"][0]["pose"]["orientation"]["x"], 6.0);
  ASSERT_EQ(json["poses"][0]["pose"]["orientation"]["y"], 7.0);
  ASSERT_EQ(json["poses"][0]["pose"]["orientation"]["z"], 8.0);
  ASSERT_EQ(json["poses"][0]["pose"]["orientation"]["w"], 9.0);
  ASSERT_EQ(json["poses"][1]["__type"], "geometry_msgs::msg::PoseStamped");
  ASSERT_EQ(json["poses"][1]["header"]["__type"], "std_msgs::msg::Header");
  ASSERT_EQ(
    json["poses"][1]["header"]["stamp"]["__type"],
    "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["poses"][1]["header"]["stamp"]["sec"], 10);
  ASSERT_EQ(json["poses"][1]["header"]["stamp"]["nanosec"], 11);
  ASSERT_EQ(json["poses"][1]["header"]["frame_id"], "odom");
  ASSERT_EQ(json["poses"][1]["pose"]["__type"], "geometry_msgs::msg::Pose");
  ASSERT_EQ(
    json["poses"][1]["pose"]["position"]["__type"],
    "geometry_msgs::msg::Point");
  ASSERT_EQ(json["poses"][1]["pose"]["position"]["x"], 12.0);
  ASSERT_EQ(json["poses"][1]["pose"]["position"]["y"], 13.0);
  ASSERT_EQ(json["poses"][1]["pose"]["position"]["z"], 14.0);
  ASSERT_EQ(
    json["poses"][1]["pose"]["orientation"]["__type"],
    "geometry_msgs::msg::Quaternion");
  ASSERT_EQ(json["poses"][1]["pose"]["orientation"]["x"], 15.0);
  ASSERT_EQ(json["poses"][1]["pose"]["orientation"]["y"], 16.0);
  ASSERT_EQ(json["poses"][1]["pose"]["orientation"]["z"], 17.0);
  ASSERT_EQ(json["poses"][1]["pose"]["orientation"]["w"], 18.0);

  // Check the two-ways transform, i.e. "from_json"
  // auto pose_stamped_vector_test2 =
  //   exporter.fromJson(json["poses"])->first.cast<std::vector<geometry_msgs::msg::PoseStamped>>();
  // ASSERT_EQ(pose_stamped_vector_test[0].header.stamp.sec,
  //   pose_stamped_vector_test2[0].header.stamp.sec);
  // ASSERT_EQ(pose_stamped_vector_test[0].header.stamp.nanosec,
  //   pose_stamped_vector_test2[0].header.stamp.nanosec);
  // ASSERT_EQ(pose_stamped_vector_test[0].header.frame_id,
  //   pose_stamped_vector_test2[0].header.frame_id);
  // ASSERT_EQ(pose_stamped_vector_test[0].pose.position.x,
  //   pose_stamped_vector_test2[0].pose.position.x);
  // ASSERT_EQ(pose_stamped_vector_test[0].pose.position.y,
  //   pose_stamped_vector_test2[0].pose.position.y);
  // ASSERT_EQ(pose_stamped_vector_test[0].pose.position.z,
  //   pose_stamped_vector_test2[0].pose.position.z);
  // ASSERT_EQ(pose_stamped_vector_test[0].pose.orientation.x,
  //   pose_stamped_vector_test2[0].pose.orientation.x);
  // ASSERT_EQ(pose_stamped_vector_test[0].pose.orientation.y,
  //   pose_stamped_vector_test2[0].pose.orientation.y);
  // ASSERT_EQ(pose_stamped_vector_test[0].pose.orientation.z,
  //   pose_stamped_vector_test2[0].pose.orientation.z);
  // ASSERT_EQ(pose_stamped_vector_test[0].pose.orientation.w,
  //   pose_stamped_vector_test2[0].pose.orientation.w);
  // ASSERT_EQ(pose_stamped_vector_test[1].header.stamp.sec,
  //   pose_stamped_vector_test2[1].header.stamp.sec);
  // ASSERT_EQ(pose_stamped_vector_test[1].header.stamp.nanosec,
  //   pose_stamped_vector_test2[1].header.stamp.nanosec);
  // ASSERT_EQ(pose_stamped_vector_test[1].header.frame_id,
  //   pose_stamped_vector_test2[1].header.frame_id);
  // ASSERT_EQ(pose_stamped_vector_test[1].pose.position.x,
  //   pose_stamped_vector_test2[1].pose.position.x);
  // ASSERT_EQ(pose_stamped_vector_test[1].pose.position.y,
  //   pose_stamped_vector_test2[1].pose.position.y);
  // ASSERT_EQ(pose_stamped_vector_test[1].pose.position.z,
  //   pose_stamped_vector_test2[1].pose.position.z);
  // ASSERT_EQ(pose_stamped_vector_test[1].pose.orientation.x,
  //   pose_stamped_vector_test2[1].pose.orientation.x);
  // ASSERT_EQ(pose_stamped_vector_test[1].pose.orientation.y,
  //   pose_stamped_vector_test2[1].pose.orientation.y);
  // ASSERT_EQ(pose_stamped_vector_test[1].pose.orientation.z,
  //   pose_stamped_vector_test2[1].pose.orientation.z);
  // ASSERT_EQ(pose_stamped_vector_test[1].pose.orientation.w,
  //   pose_stamped_vector_test2[1].pose.orientation.w);

  // // Convert from string
  // std::vector<geometry_msgs::msg::PoseStamped> pose_stamped_vector_test3;
  // auto const test_json =
  //   R"(json:
  //     {
  //       "poses": [
  //         {
  //           "__type": "geometry_msgs::msg::PoseStamped",
  //           "header": {
  //             "__type": "std_msgs::msg::Header",
  //             "stamp": {"__type": "builtin_interfaces::msg::Time", "sec": 1, "nanosec": 2},
  //             "frame_id": "map"
  //           },
  //           "pose": {
  //             "__type": "geometry_msgs::msg::Pose",
  //             "position": {
  //               "__type": "geometry_msgs::msg::Point",
  //               "x": 3.0, "y": 4.0, "z": 5.0
  //             },
  //             "orientation": {
  //               "__type": "geometry_msgs::msg::Quaternion",
  //               "x": 6.0, "y": 7.0, "z": 8.0, "w": 9.0
  //             }
  //           }
  //         },
  //         {
  //           "__type": "geometry_msgs::msg::PoseStamped",
  //           "header": {
  //             "__type": "std_msgs::msg::Header",
  //             "stamp": {"__type": "builtin_interfaces::msg::Time", "sec": 10, "nanosec": 11},
  //             "frame_id": "odom"
  //           },
  //           "pose": {
  //             "__type": "geometry_msgs::msg::Pose",
  //             "position": {
  //               "__type": "geometry_msgs::msg::Point",
  //               "x": 12.0, "y": 13.0, "z": 14.0
  //             },
  //             "orientation": {
  //               "__type": "geometry_msgs::msg::Quaternion",
  //               "x": 15.0, "y": 16.0, "z": 17.0, "w": 18.0
  //             }
  //           }
  //         }
  //       ]
  //     }
  //   )";
  // ASSERT_NO_THROW(pose_stamped_vector_test3 =
  //   BT::convertFromString<std::vector<geometry_msgs::msg::PoseStamped>>(test_json));
  // ASSERT_EQ(pose_stamped_vector_test[0].header, pose_stamped_vector_test3[0].header);
  // ASSERT_EQ(pose_stamped_vector_test[0].pose.position,
  //   pose_stamped_vector_test3[0].pose.position);
  // ASSERT_EQ(pose_stamped_vector_test[0].pose.orientation,
  //   pose_stamped_vector_test3[0].pose.orientation);
  // ASSERT_EQ(pose_stamped_vector_test[1].header, pose_stamped_vector_test3[1].header);
  // ASSERT_EQ(pose_stamped_vector_test[1].pose.position,
  //   pose_stamped_vector_test3[1].pose.position);
  // ASSERT_EQ(pose_stamped_vector_test[1].pose.orientation,
  //   pose_stamped_vector_test3[1].pose.orientation);
}

TEST_F(JsonTest, test_goals)
{
  BT::JsonExporter & exporter = BT::JsonExporter::get();

  nav_msgs::msg::Goals goals_test;
  goals_test.header.stamp.sec = 1;
  goals_test.header.stamp.nanosec = 2;
  goals_test.header.frame_id = "map";
  goals_test.goals.resize(2);
  goals_test.goals[0].header.stamp.sec = 3;
  goals_test.goals[0].header.stamp.nanosec = 4;
  goals_test.goals[0].header.frame_id = "map";
  goals_test.goals[0].pose.position.x = 5.0;
  goals_test.goals[0].pose.position.y = 6.0;
  goals_test.goals[0].pose.position.z = 7.0;
  goals_test.goals[0].pose.orientation.x = 8.0;
  goals_test.goals[0].pose.orientation.y = 9.0;
  goals_test.goals[0].pose.orientation.z = 10.0;
  goals_test.goals[0].pose.orientation.w = 11.0;

  goals_test.goals[1].header.stamp.sec = 12;
  goals_test.goals[1].header.stamp.nanosec = 13;
  goals_test.goals[1].header.frame_id = "odom";
  goals_test.goals[1].pose.position.x = 14.0;
  goals_test.goals[1].pose.position.y = 15.0;
  goals_test.goals[1].pose.position.z = 16.0;
  goals_test.goals[1].pose.orientation.x = 17.0;
  goals_test.goals[1].pose.orientation.y = 18.0;
  goals_test.goals[1].pose.orientation.z = 19.0;
  goals_test.goals[1].pose.orientation.w = 20.0;

  nlohmann::json json;
  exporter.toJson(BT::Any(goals_test), json["goals"]);

  std::cout << json.dump(2) << std::endl;

  ASSERT_EQ(json["goals"]["__type"], "nav_msgs::msg::Goals");
  ASSERT_EQ(json["goals"]["header"]["__type"], "std_msgs::msg::Header");
  ASSERT_EQ(json["goals"]["header"]["stamp"]["__type"], "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["goals"]["header"]["stamp"]["sec"], 1);
  ASSERT_EQ(json["goals"]["header"]["stamp"]["nanosec"], 2);
  ASSERT_EQ(json["goals"]["header"]["frame_id"], "map");
  ASSERT_EQ(json["goals"]["goals"][0]["__type"], "geometry_msgs::msg::PoseStamped");
  ASSERT_EQ(json["goals"]["goals"][0]["header"]["__type"], "std_msgs::msg::Header");
  ASSERT_EQ(
    json["goals"]["goals"][0]["header"]["stamp"]["__type"],
    "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["goals"]["goals"][0]["header"]["stamp"]["sec"], 3);
  ASSERT_EQ(json["goals"]["goals"][0]["header"]["stamp"]["nanosec"], 4);
  ASSERT_EQ(json["goals"]["goals"][0]["header"]["frame_id"], "map");
  ASSERT_EQ(json["goals"]["goals"][0]["pose"]["__type"], "geometry_msgs::msg::Pose");
  ASSERT_EQ(json["goals"]["goals"][0]["pose"]["position"]["__type"], "geometry_msgs::msg::Point");
  ASSERT_EQ(json["goals"]["goals"][0]["pose"]["position"]["x"], 5.0);
  ASSERT_EQ(json["goals"]["goals"][0]["pose"]["position"]["y"], 6.0);
  ASSERT_EQ(json["goals"]["goals"][0]["pose"]["position"]["z"], 7.0);
  ASSERT_EQ(
    json["goals"]["goals"][0]["pose"]["orientation"]["__type"],
    "geometry_msgs::msg::Quaternion");
  ASSERT_EQ(json["goals"]["goals"][0]["pose"]["orientation"]["x"], 8.0);
  ASSERT_EQ(json["goals"]["goals"][0]["pose"]["orientation"]["y"], 9.0);
  ASSERT_EQ(json["goals"]["goals"][0]["pose"]["orientation"]["z"], 10.0);
  ASSERT_EQ(json["goals"]["goals"][0]["pose"]["orientation"]["w"], 11.0);
  ASSERT_EQ(json["goals"]["goals"][1]["__type"], "geometry_msgs::msg::PoseStamped");
  ASSERT_EQ(json["goals"]["goals"][1]["header"]["__type"], "std_msgs::msg::Header");
  ASSERT_EQ(
    json["goals"]["goals"][1]["header"]["stamp"]["__type"],
    "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["goals"]["goals"][1]["header"]["stamp"]["sec"], 12);
  ASSERT_EQ(json["goals"]["goals"][1]["header"]["stamp"]["nanosec"], 13);
  ASSERT_EQ(json["goals"]["goals"][1]["header"]["frame_id"], "odom");
  ASSERT_EQ(json["goals"]["goals"][1]["pose"]["__type"], "geometry_msgs::msg::Pose");
  ASSERT_EQ(json["goals"]["goals"][1]["pose"]["position"]["__type"], "geometry_msgs::msg::Point");
  ASSERT_EQ(json["goals"]["goals"][1]["pose"]["position"]["x"], 14.0);
  ASSERT_EQ(json["goals"]["goals"][1]["pose"]["position"]["y"], 15.0);
  ASSERT_EQ(json["goals"]["goals"][1]["pose"]["position"]["z"], 16.0);
  ASSERT_EQ(
    json["goals"]["goals"][1]["pose"]["orientation"]["__type"],
    "geometry_msgs::msg::Quaternion");
  ASSERT_EQ(json["goals"]["goals"][1]["pose"]["orientation"]["x"], 17.0);
  ASSERT_EQ(json["goals"]["goals"][1]["pose"]["orientation"]["y"], 18.0);
  ASSERT_EQ(json["goals"]["goals"][1]["pose"]["orientation"]["z"], 19.0);
  ASSERT_EQ(json["goals"]["goals"][1]["pose"]["orientation"]["w"], 20.0);

  // Check the two-ways transform, i.e. "from_json"
  auto goals_test2 = exporter.fromJson(json["goals"])->first.cast<nav_msgs::msg::Goals>();

  ASSERT_EQ(goals_test.goals[0].header.stamp.sec, goals_test2.goals[0].header.stamp.sec);
  ASSERT_EQ(goals_test.goals[0].header.stamp.nanosec, goals_test2.goals[0].header.stamp.nanosec);
  ASSERT_EQ(goals_test.goals[0].header.frame_id, goals_test2.goals[0].header.frame_id);
  ASSERT_EQ(goals_test.goals[0].pose.position.x, goals_test2.goals[0].pose.position.x);
  ASSERT_EQ(goals_test.goals[0].pose.position.y, goals_test2.goals[0].pose.position.y);
  ASSERT_EQ(goals_test.goals[0].pose.position.z, goals_test2.goals[0].pose.position.z);
  ASSERT_EQ(goals_test.goals[0].pose.orientation.x, goals_test2.goals[0].pose.orientation.x);
  ASSERT_EQ(goals_test.goals[0].pose.orientation.y, goals_test2.goals[0].pose.orientation.y);
  ASSERT_EQ(goals_test.goals[0].pose.orientation.z, goals_test2.goals[0].pose.orientation.z);
  ASSERT_EQ(goals_test.goals[0].pose.orientation.w, goals_test2.goals[0].pose.orientation.w);
  ASSERT_EQ(goals_test.goals[1].header.stamp.sec, goals_test2.goals[1].header.stamp.sec);
  ASSERT_EQ(goals_test.goals[1].header.stamp.nanosec, goals_test2.goals[1].header.stamp.nanosec);
  ASSERT_EQ(goals_test.goals[1].header.frame_id, goals_test2.goals[1].header.frame_id);
  ASSERT_EQ(goals_test.goals[1].pose.position.x, goals_test2.goals[1].pose.position.x);
  ASSERT_EQ(goals_test.goals[1].pose.position.y, goals_test2.goals[1].pose.position.y);
  ASSERT_EQ(goals_test.goals[1].pose.position.z, goals_test2.goals[1].pose.position.z);
  ASSERT_EQ(goals_test.goals[1].pose.orientation.x, goals_test2.goals[1].pose.orientation.x);
  ASSERT_EQ(goals_test.goals[1].pose.orientation.y, goals_test2.goals[1].pose.orientation.y);
  ASSERT_EQ(goals_test.goals[1].pose.orientation.z, goals_test2.goals[1].pose.orientation.z);
  ASSERT_EQ(goals_test.goals[1].pose.orientation.w, goals_test2.goals[1].pose.orientation.w);

  // Convert from string
  nav_msgs::msg::Goals goals_test3;
  auto const test_json =
    R"(json:
      {
        "__type": "nav_msgs::msg::Goals",
        "header": {
          "__type": "std_msgs::msg::Header",
          "stamp": {"__type": "builtin_interfaces::msg::Time", "sec": 1, "nanosec": 2},
          "frame_id": "map"
        },
        "goals": [
          {
            "__type": "geometry_msgs::msg::PoseStamped",
            "header": {
              "__type": "std_msgs::msg::Header",
              "stamp": {"__type": "builtin_interfaces::msg::Time", "sec": 3, "nanosec": 4},
              "frame_id": "map"
            },
            "pose": {
              "__type": "geometry_msgs::msg::Pose",
              "position": {
                "__type": "geometry_msgs::msg::Point",
                "x": 5.0, "y": 6.0, "z": 7.0
              },
              "orientation": {
                "__type": "geometry_msgs::msg::Quaternion",
                "x": 8.0, "y": 9.0, "z": 10.0, "w": 11.0
              }
            }
          },
          {
            "__type": "geometry_msgs::msg::PoseStamped",
            "header": {
              "__type": "std_msgs::msg::Header",
              "stamp": {"__type": "builtin_interfaces::msg::Time", "sec": 12, "nanosec": 13},
              "frame_id": "odom"
            },
            "pose": {
              "__type": "geometry_msgs::msg::Pose",
              "position": {
                "__type": "geometry_msgs::msg::Point",
                "x": 14.0, "y": 15.0, "z": 16.0
              },
              "orientation": {
                "__type": "geometry_msgs::msg::Quaternion",
                "x": 17.0, "y": 18.0, "z": 19.0, "w": 20.0
              }
            }
          }
        ]
      }
    )";
  ASSERT_NO_THROW(
    goals_test3 =
    BT::convertFromString<nav_msgs::msg::Goals>(test_json));
  ASSERT_EQ(goals_test.goals[0].header, goals_test3.goals[0].header);
  ASSERT_EQ(goals_test.goals[0].pose.position, goals_test3.goals[0].pose.position);
  ASSERT_EQ(goals_test.goals[0].pose.orientation, goals_test3.goals[0].pose.orientation);
  ASSERT_EQ(goals_test.goals[1].header, goals_test3.goals[1].header);
  ASSERT_EQ(goals_test.goals[1].pose.position, goals_test3.goals[1].pose.position);
  ASSERT_EQ(goals_test.goals[1].pose.orientation, goals_test3.goals[1].pose.orientation);
}

TEST_F(JsonTest, test_path)
{
  BT::JsonExporter & exporter = BT::JsonExporter::get();

  nav_msgs::msg::Path path_test;
  path_test.header.stamp.sec = 1;
  path_test.header.stamp.nanosec = 2;
  path_test.header.frame_id = "map";
  path_test.poses.resize(2);
  path_test.poses[0].header.stamp.sec = 3;
  path_test.poses[0].header.stamp.nanosec = 4;
  path_test.poses[0].header.frame_id = "map";
  path_test.poses[0].pose.position.x = 5.0;
  path_test.poses[0].pose.position.y = 6.0;
  path_test.poses[0].pose.position.z = 7.0;
  path_test.poses[0].pose.orientation.x = 8.0;
  path_test.poses[0].pose.orientation.y = 9.0;
  path_test.poses[0].pose.orientation.z = 10.0;
  path_test.poses[0].pose.orientation.w = 11.0;

  path_test.poses[1].header.stamp.sec = 12;
  path_test.poses[1].header.stamp.nanosec = 13;
  path_test.poses[1].header.frame_id = "odom";
  path_test.poses[1].pose.position.x = 14.0;
  path_test.poses[1].pose.position.y = 15.0;
  path_test.poses[1].pose.position.z = 16.0;
  path_test.poses[1].pose.orientation.x = 17.0;
  path_test.poses[1].pose.orientation.y = 18.0;
  path_test.poses[1].pose.orientation.z = 19.0;
  path_test.poses[1].pose.orientation.w = 20.0;

  nlohmann::json json;
  exporter.toJson(BT::Any(path_test), json["path"]);

  std::cout << json.dump(2) << std::endl;

  ASSERT_EQ(json["path"]["__type"], "nav_msgs::msg::Path");
  ASSERT_EQ(json["path"]["header"]["__type"], "std_msgs::msg::Header");
  ASSERT_EQ(json["path"]["header"]["stamp"]["__type"], "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["path"]["header"]["stamp"]["sec"], 1);
  ASSERT_EQ(json["path"]["header"]["stamp"]["nanosec"], 2);
  ASSERT_EQ(json["path"]["header"]["frame_id"], "map");
  ASSERT_EQ(json["path"]["poses"][0]["__type"], "geometry_msgs::msg::PoseStamped");
  ASSERT_EQ(json["path"]["poses"][0]["header"]["__type"], "std_msgs::msg::Header");
  ASSERT_EQ(
    json["path"]["poses"][0]["header"]["stamp"]["__type"],
    "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["path"]["poses"][0]["header"]["stamp"]["sec"], 3);
  ASSERT_EQ(json["path"]["poses"][0]["header"]["stamp"]["nanosec"], 4);
  ASSERT_EQ(json["path"]["poses"][0]["header"]["frame_id"], "map");
  ASSERT_EQ(json["path"]["poses"][0]["pose"]["__type"], "geometry_msgs::msg::Pose");
  ASSERT_EQ(
    json["path"]["poses"][0]["pose"]["position"]["__type"],
    "geometry_msgs::msg::Point");
  ASSERT_EQ(json["path"]["poses"][0]["pose"]["position"]["x"], 5.0);
  ASSERT_EQ(json["path"]["poses"][0]["pose"]["position"]["y"], 6.0);
  ASSERT_EQ(json["path"]["poses"][0]["pose"]["position"]["z"], 7.0);
  ASSERT_EQ(
    json["path"]["poses"][0]["pose"]["orientation"]["__type"],
    "geometry_msgs::msg::Quaternion");
  ASSERT_EQ(json["path"]["poses"][0]["pose"]["orientation"]["x"], 8.0);
  ASSERT_EQ(json["path"]["poses"][0]["pose"]["orientation"]["y"], 9.0);
  ASSERT_EQ(json["path"]["poses"][0]["pose"]["orientation"]["z"], 10.0);
  ASSERT_EQ(json["path"]["poses"][0]["pose"]["orientation"]["w"], 11.0);
  ASSERT_EQ(json["path"]["poses"][1]["__type"], "geometry_msgs::msg::PoseStamped");
  ASSERT_EQ(json["path"]["poses"][1]["header"]["__type"], "std_msgs::msg::Header");
  ASSERT_EQ(
    json["path"]["poses"][1]["header"]["stamp"]["__type"],
    "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["path"]["poses"][1]["header"]["stamp"]["sec"], 12);
  ASSERT_EQ(json["path"]["poses"][1]["header"]["stamp"]["nanosec"], 13);
  ASSERT_EQ(json["path"]["poses"][1]["header"]["frame_id"], "odom");
  ASSERT_EQ(json["path"]["poses"][1]["pose"]["__type"], "geometry_msgs::msg::Pose");
  ASSERT_EQ(
    json["path"]["poses"][1]["pose"]["position"]["__type"],
    "geometry_msgs::msg::Point");
  ASSERT_EQ(json["path"]["poses"][1]["pose"]["position"]["x"], 14.0);
  ASSERT_EQ(json["path"]["poses"][1]["pose"]["position"]["y"], 15.0);
  ASSERT_EQ(json["path"]["poses"][1]["pose"]["position"]["z"], 16.0);
  ASSERT_EQ(
    json["path"]["poses"][1]["pose"]["orientation"]["__type"],
    "geometry_msgs::msg::Quaternion");
  ASSERT_EQ(json["path"]["poses"][1]["pose"]["orientation"]["x"], 17.0);
  ASSERT_EQ(json["path"]["poses"][1]["pose"]["orientation"]["y"], 18.0);
  ASSERT_EQ(json["path"]["poses"][1]["pose"]["orientation"]["z"], 19.0);
  ASSERT_EQ(json["path"]["poses"][1]["pose"]["orientation"]["w"], 20.0);

  // Check the two-ways transform, i.e. "from_json"
  auto path_test2 = exporter.fromJson(json["path"])->first.cast<nav_msgs::msg::Path>();
  ASSERT_EQ(path_test.header.stamp.sec, path_test2.header.stamp.sec);
  ASSERT_EQ(path_test.header.stamp.nanosec, path_test2.header.stamp.nanosec);
  ASSERT_EQ(path_test.header.frame_id, path_test2.header.frame_id);
  ASSERT_EQ(path_test.poses[0].header.stamp.sec, path_test2.poses[0].header.stamp.sec);
  ASSERT_EQ(path_test.poses[0].header.stamp.nanosec, path_test2.poses[0].header.stamp.nanosec);
  ASSERT_EQ(path_test.poses[0].header.frame_id, path_test2.poses[0].header.frame_id);
  ASSERT_EQ(path_test.poses[0].pose.position.x, path_test2.poses[0].pose.position.x);
  ASSERT_EQ(path_test.poses[0].pose.position.y, path_test2.poses[0].pose.position.y);
  ASSERT_EQ(path_test.poses[0].pose.position.z, path_test2.poses[0].pose.position.z);
  ASSERT_EQ(path_test.poses[0].pose.orientation.x, path_test2.poses[0].pose.orientation.x);
  ASSERT_EQ(path_test.poses[0].pose.orientation.y, path_test2.poses[0].pose.orientation.y);
  ASSERT_EQ(path_test.poses[0].pose.orientation.z, path_test2.poses[0].pose.orientation.z);
  ASSERT_EQ(path_test.poses[0].pose.orientation.w, path_test2.poses[0].pose.orientation.w);
  ASSERT_EQ(path_test.poses[1].header.stamp.sec, path_test2.poses[1].header.stamp.sec);
  ASSERT_EQ(path_test.poses[1].header.stamp.nanosec, path_test2.poses[1].header.stamp.nanosec);
  ASSERT_EQ(path_test.poses[1].header.frame_id, path_test2.poses[1].header.frame_id);
  ASSERT_EQ(path_test.poses[1].pose.position.x, path_test2.poses[1].pose.position.x);
  ASSERT_EQ(path_test.poses[1].pose.position.y, path_test2.poses[1].pose.position.y);
  ASSERT_EQ(path_test.poses[1].pose.position.z, path_test2.poses[1].pose.position.z);
  ASSERT_EQ(path_test.poses[1].pose.orientation.x, path_test2.poses[1].pose.orientation.x);
  ASSERT_EQ(path_test.poses[1].pose.orientation.y, path_test2.poses[1].pose.orientation.y);
  ASSERT_EQ(path_test.poses[1].pose.orientation.z, path_test2.poses[1].pose.orientation.z);
  ASSERT_EQ(path_test.poses[1].pose.orientation.w, path_test2.poses[1].pose.orientation.w);

  // Convert from string
  nav_msgs::msg::Path path_test3;
  auto const test_json =
    R"(json:
      {
        "__type": "nav_msgs::msg::Path",
        "header": {
          "__type": "std_msgs::msg::Header",
          "stamp": {"__type": "builtin_interfaces::msg::Time", "sec": 1, "nanosec": 2},
          "frame_id": "map"
        },
        "poses": [
          {
            "__type": "geometry_msgs::msg::PoseStamped",
            "header": {
              "__type": "std_msgs::msg::Header",
              "stamp": {"__type": "builtin_interfaces::msg::Time", "sec": 3, "nanosec": 4},
              "frame_id": "map"
            },
            "pose": {
              "__type": "geometry_msgs::msg::Pose",
              "position": {
                "__type": "geometry_msgs::msg::Point",
                "x": 5.0, "y": 6.0, "z": 7.0
              },
              "orientation": {
                "__type": "geometry_msgs::msg::Quaternion",
                "x": 8.0, "y": 9.0, "z": 10.0, "w": 11.0
              }
            }
          },
          {
            "__type": "geometry_msgs::msg::PoseStamped",
            "header": {
              "__type": "std_msgs::msg::Header",
              "stamp": {"__type": "builtin_interfaces::msg::Time", "sec": 12, "nanosec": 13},
              "frame_id": "odom"
            },
            "pose": {
              "__type": "geometry_msgs::msg::Pose",
              "position": {
                "__type": "geometry_msgs::msg::Point",
                "x": 14.0, "y": 15.0, "z": 16.0
              },
              "orientation": {
                "__type": "geometry_msgs::msg::Quaternion",
                "x": 17.0, "y": 18.0, "z": 19.0, "w": 20.0
              }
            }
          }
        ]
      }
    )";
  ASSERT_NO_THROW(
    path_test3 =
    BT::convertFromString<nav_msgs::msg::Path>(test_json));
  ASSERT_EQ(path_test.header, path_test3.header);
  ASSERT_EQ(path_test.poses[0].header, path_test3.poses[0].header);
  ASSERT_EQ(path_test.poses[0].pose.position, path_test3.poses[0].pose.position);
  ASSERT_EQ(path_test.poses[0].pose.orientation, path_test3.poses[0].pose.orientation);
  ASSERT_EQ(path_test.poses[1].header, path_test3.poses[1].header);
  ASSERT_EQ(path_test.poses[1].pose.position, path_test3.poses[1].pose.position);
  ASSERT_EQ(path_test.poses[1].pose.orientation, path_test3.poses[1].pose.orientation);
}

TEST_F(JsonTest, test_waypoint_status)
{
  BT::JsonExporter & exporter = BT::JsonExporter::get();

  nav2_msgs::msg::WaypointStatus waypoint_status_test;
  waypoint_status_test.waypoint_status = 1;
  waypoint_status_test.waypoint_index = 2;
  waypoint_status_test.waypoint_pose.header.stamp.sec = 1;
  waypoint_status_test.waypoint_pose.header.stamp.nanosec = 2;
  waypoint_status_test.waypoint_pose.header.frame_id = "map";
  waypoint_status_test.waypoint_pose.pose.position.x = 3.0;
  waypoint_status_test.waypoint_pose.pose.position.y = 4.0;
  waypoint_status_test.waypoint_pose.pose.position.z = 5.0;
  waypoint_status_test.waypoint_pose.pose.orientation.x = 6.0;
  waypoint_status_test.waypoint_pose.pose.orientation.y = 7.0;
  waypoint_status_test.waypoint_pose.pose.orientation.z = 8.0;
  waypoint_status_test.waypoint_pose.pose.orientation.w = 9.0;
  waypoint_status_test.error_code = 10;
  waypoint_status_test.error_msg = "error";

  nlohmann::json json;
  exporter.toJson(BT::Any(waypoint_status_test), json["waypoint_status"]);

  std::cout << json.dump(2) << std::endl;

  ASSERT_EQ(json["waypoint_status"]["__type"], "nav2_msgs::msg::WaypointStatus");
  ASSERT_EQ(json["waypoint_status"]["waypoint_status"], 1);
  ASSERT_EQ(json["waypoint_status"]["waypoint_index"], 2);
  ASSERT_EQ(
    json["waypoint_status"]["waypoint_pose"]["__type"],
    "geometry_msgs::msg::PoseStamped");
  ASSERT_EQ(
    json["waypoint_status"]["waypoint_pose"]["header"]["__type"],
    "std_msgs::msg::Header");
  ASSERT_EQ(
    json["waypoint_status"]["waypoint_pose"]["header"]["stamp"]["__type"],
    "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["waypoint_status"]["waypoint_pose"]["header"]["stamp"]["sec"], 1);
  ASSERT_EQ(json["waypoint_status"]["waypoint_pose"]["header"]["stamp"]["nanosec"], 2);
  ASSERT_EQ(json["waypoint_status"]["waypoint_pose"]["header"]["frame_id"], "map");
  ASSERT_EQ(
    json["waypoint_status"]["waypoint_pose"]["pose"]["__type"],
    "geometry_msgs::msg::Pose");
  ASSERT_EQ(
    json["waypoint_status"]["waypoint_pose"]["pose"]["position"]["__type"],
    "geometry_msgs::msg::Point");
  ASSERT_EQ(json["waypoint_status"]["waypoint_pose"]["pose"]["position"]["x"], 3.0);
  ASSERT_EQ(json["waypoint_status"]["waypoint_pose"]["pose"]["position"]["y"], 4.0);
  ASSERT_EQ(json["waypoint_status"]["waypoint_pose"]["pose"]["position"]["z"], 5.0);
  ASSERT_EQ(
    json["waypoint_status"]["waypoint_pose"]["pose"]["orientation"]["__type"],
    "geometry_msgs::msg::Quaternion");
  ASSERT_EQ(json["waypoint_status"]["waypoint_pose"]["pose"]["orientation"]["x"], 6.0);
  ASSERT_EQ(json["waypoint_status"]["waypoint_pose"]["pose"]["orientation"]["y"], 7.0);
  ASSERT_EQ(json["waypoint_status"]["waypoint_pose"]["pose"]["orientation"]["z"], 8.0);
  ASSERT_EQ(json["waypoint_status"]["waypoint_pose"]["pose"]["orientation"]["w"], 9.0);
  ASSERT_EQ(json["waypoint_status"]["error_code"], 10);
  ASSERT_EQ(json["waypoint_status"]["error_msg"], "error");

  // Check the two-ways transform, i.e. "from_json"
  auto waypoint_status_test2 =
    exporter.fromJson(json["waypoint_status"])->first.cast<nav2_msgs::msg::WaypointStatus>();
  ASSERT_EQ(waypoint_status_test.waypoint_status, waypoint_status_test2.waypoint_status);
  ASSERT_EQ(waypoint_status_test.waypoint_index, waypoint_status_test2.waypoint_index);
  ASSERT_EQ(
    waypoint_status_test.waypoint_pose.header.stamp.sec,
    waypoint_status_test2.waypoint_pose.header.stamp.sec);
  ASSERT_EQ(
    waypoint_status_test.waypoint_pose.header.stamp.nanosec,
    waypoint_status_test2.waypoint_pose.header.stamp.nanosec);
  ASSERT_EQ(
    waypoint_status_test.waypoint_pose.header.frame_id,
    waypoint_status_test2.waypoint_pose.header.frame_id);
  ASSERT_EQ(
    waypoint_status_test.waypoint_pose.pose.position.x,
    waypoint_status_test2.waypoint_pose.pose.position.x);
  ASSERT_EQ(
    waypoint_status_test.waypoint_pose.pose.position.y,
    waypoint_status_test2.waypoint_pose.pose.position.y);
  ASSERT_EQ(
    waypoint_status_test.waypoint_pose.pose.position.z,
    waypoint_status_test2.waypoint_pose.pose.position.z);
  ASSERT_EQ(
    waypoint_status_test.waypoint_pose.pose.orientation.x,
    waypoint_status_test2.waypoint_pose.pose.orientation.x);
  ASSERT_EQ(
    waypoint_status_test.waypoint_pose.pose.orientation.y,
    waypoint_status_test2.waypoint_pose.pose.orientation.y);
  ASSERT_EQ(
    waypoint_status_test.waypoint_pose.pose.orientation.z,
    waypoint_status_test2.waypoint_pose.pose.orientation.z);
  ASSERT_EQ(
    waypoint_status_test.waypoint_pose.pose.orientation.w,
    waypoint_status_test2.waypoint_pose.pose.orientation.w);
  ASSERT_EQ(waypoint_status_test.error_code, waypoint_status_test2.error_code);
  ASSERT_EQ(waypoint_status_test.error_msg, waypoint_status_test2.error_msg);

  // Convert from string
  nav2_msgs::msg::WaypointStatus waypoint_status_test3;
  auto const test_json =
    R"(json:
      {
        "__type": "nav2_msgs::msg::WaypointStatus",
        "waypoint_status": 1,
        "waypoint_index": 2,
        "waypoint_pose": {
          "__type": "geometry_msgs::msg::PoseStamped",
          "header": {
            "__type": "std_msgs::msg::Header",
            "stamp": {"__type": "builtin_interfaces::msg::Time", "sec": 1, "nanosec": 2},
            "frame_id": "map"
          },
          "pose": {
            "__type": "geometry_msgs::msg::Pose",
            "position": {
              "__type": "geometry_msgs::msg::Point",
              "x": 3.0, "y": 4.0, "z": 5.0
            },
            "orientation": {
              "__type": "geometry_msgs::msg::Quaternion",
              "x": 6.0, "y": 7.0, "z": 8.0, "w": 9.0
            }
          }
        },
        "error_code": 10,
        "error_msg": "error"
      }
    )";
  ASSERT_NO_THROW(
    waypoint_status_test3 =
    BT::convertFromString<nav2_msgs::msg::WaypointStatus>(test_json));
  ASSERT_EQ(waypoint_status_test.waypoint_status, waypoint_status_test3.waypoint_status);
  ASSERT_EQ(waypoint_status_test.waypoint_index, waypoint_status_test3.waypoint_index);
  ASSERT_EQ(waypoint_status_test.waypoint_pose, waypoint_status_test3.waypoint_pose);
  ASSERT_EQ(waypoint_status_test.error_code, waypoint_status_test3.error_code);
  ASSERT_EQ(waypoint_status_test.error_msg, waypoint_status_test3.error_msg);
}

TEST_F(JsonTest, test_waypoint_status_vector)
{
  BT::JsonExporter & exporter = BT::JsonExporter::get();

  std::vector<nav2_msgs::msg::WaypointStatus> waypoint_status_vector_test;
  waypoint_status_vector_test.resize(2);
  waypoint_status_vector_test[0].waypoint_status = 1;
  waypoint_status_vector_test[0].waypoint_index = 2;
  waypoint_status_vector_test[0].waypoint_pose.header.stamp.sec = 1;
  waypoint_status_vector_test[0].waypoint_pose.header.stamp.nanosec = 2;
  waypoint_status_vector_test[0].waypoint_pose.header.frame_id = "map";
  waypoint_status_vector_test[0].waypoint_pose.pose.position.x = 3.0;
  waypoint_status_vector_test[0].waypoint_pose.pose.position.y = 4.0;
  waypoint_status_vector_test[0].waypoint_pose.pose.position.z = 5.0;
  waypoint_status_vector_test[0].waypoint_pose.pose.orientation.x = 6.0;
  waypoint_status_vector_test[0].waypoint_pose.pose.orientation.y = 7.0;
  waypoint_status_vector_test[0].waypoint_pose.pose.orientation.z = 8.0;
  waypoint_status_vector_test[0].waypoint_pose.pose.orientation.w = 9.0;
  waypoint_status_vector_test[0].error_code = 10;
  waypoint_status_vector_test[0].error_msg = "error";

  waypoint_status_vector_test[1].waypoint_status = 11;
  waypoint_status_vector_test[1].waypoint_index = 12;
  waypoint_status_vector_test[1].waypoint_pose.header.stamp.sec = 13;
  waypoint_status_vector_test[1].waypoint_pose.header.stamp.nanosec = 14;
  waypoint_status_vector_test[1].waypoint_pose.header.frame_id = "odom";
  waypoint_status_vector_test[1].waypoint_pose.pose.position.x = 15.0;
  waypoint_status_vector_test[1].waypoint_pose.pose.position.y = 16.0;
  waypoint_status_vector_test[1].waypoint_pose.pose.position.z = 17.0;
  waypoint_status_vector_test[1].waypoint_pose.pose.orientation.x = 18.0;
  waypoint_status_vector_test[1].waypoint_pose.pose.orientation.y = 19.0;
  waypoint_status_vector_test[1].waypoint_pose.pose.orientation.z = 20.0;
  waypoint_status_vector_test[1].waypoint_pose.pose.orientation.w = 21.0;
  waypoint_status_vector_test[1].error_code = 22;
  waypoint_status_vector_test[1].error_msg = "error2";

  nlohmann::json json;
  exporter.toJson(BT::Any(waypoint_status_vector_test), json["waypoint_status_vector"]);
  std::cout << json.dump(2) << std::endl;

  ASSERT_EQ(json["waypoint_status_vector"][0]["__type"], "nav2_msgs::msg::WaypointStatus");
  ASSERT_EQ(json["waypoint_status_vector"][0]["waypoint_status"], 1);
  ASSERT_EQ(json["waypoint_status_vector"][0]["waypoint_index"], 2);
  ASSERT_EQ(
    json["waypoint_status_vector"][0]["waypoint_pose"]["__type"],
    "geometry_msgs::msg::PoseStamped");
  ASSERT_EQ(
    json["waypoint_status_vector"][0]["waypoint_pose"]["header"]["__type"],
    "std_msgs::msg::Header");
  ASSERT_EQ(
    json["waypoint_status_vector"][0]["waypoint_pose"]["header"]["stamp"]["__type"],
    "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["waypoint_status_vector"][0]["waypoint_pose"]["header"]["stamp"]["sec"], 1);
  ASSERT_EQ(json["waypoint_status_vector"][0]["waypoint_pose"]["header"]["stamp"]["nanosec"], 2);
  ASSERT_EQ(json["waypoint_status_vector"][0]["waypoint_pose"]["header"]["frame_id"], "map");
  ASSERT_EQ(
    json["waypoint_status_vector"][0]["waypoint_pose"]["pose"]["__type"],
    "geometry_msgs::msg::Pose");
  ASSERT_EQ(
    json["waypoint_status_vector"][0]["waypoint_pose"]["pose"]["position"]["__type"],
    "geometry_msgs::msg::Point");
  ASSERT_EQ(json["waypoint_status_vector"][0]["waypoint_pose"]["pose"]["position"]["x"], 3.0);
  ASSERT_EQ(json["waypoint_status_vector"][0]["waypoint_pose"]["pose"]["position"]["y"], 4.0);
  ASSERT_EQ(json["waypoint_status_vector"][0]["waypoint_pose"]["pose"]["position"]["z"], 5.0);
  ASSERT_EQ(
    json["waypoint_status_vector"][0]["waypoint_pose"]["pose"]["orientation"]["__type"],
    "geometry_msgs::msg::Quaternion");
  ASSERT_EQ(json["waypoint_status_vector"][0]["waypoint_pose"]["pose"]["orientation"]["x"], 6.0);
  ASSERT_EQ(json["waypoint_status_vector"][0]["waypoint_pose"]["pose"]["orientation"]["y"], 7.0);
  ASSERT_EQ(json["waypoint_status_vector"][0]["waypoint_pose"]["pose"]["orientation"]["z"], 8.0);
  ASSERT_EQ(json["waypoint_status_vector"][0]["waypoint_pose"]["pose"]["orientation"]["w"], 9.0);
  ASSERT_EQ(json["waypoint_status_vector"][0]["error_code"], 10);
  ASSERT_EQ(json["waypoint_status_vector"][0]["error_msg"], "error");
  ASSERT_EQ(json["waypoint_status_vector"][1]["__type"], "nav2_msgs::msg::WaypointStatus");
  ASSERT_EQ(json["waypoint_status_vector"][1]["waypoint_status"], 11);
  ASSERT_EQ(json["waypoint_status_vector"][1]["waypoint_index"], 12);
  ASSERT_EQ(
    json["waypoint_status_vector"][1]["waypoint_pose"]["__type"],
    "geometry_msgs::msg::PoseStamped");
  ASSERT_EQ(
    json["waypoint_status_vector"][1]["waypoint_pose"]["header"]["__type"],
    "std_msgs::msg::Header");
  ASSERT_EQ(
    json["waypoint_status_vector"][1]["waypoint_pose"]["header"]["stamp"]["__type"],
    "builtin_interfaces::msg::Time");
  ASSERT_EQ(json["waypoint_status_vector"][1]["waypoint_pose"]["header"]["stamp"]["sec"], 13);
  ASSERT_EQ(json["waypoint_status_vector"][1]["waypoint_pose"]["header"]["stamp"]["nanosec"], 14);
  ASSERT_EQ(json["waypoint_status_vector"][1]["waypoint_pose"]["header"]["frame_id"], "odom");
  ASSERT_EQ(
    json["waypoint_status_vector"][1]["waypoint_pose"]["pose"]["__type"],
    "geometry_msgs::msg::Pose");
  ASSERT_EQ(
    json["waypoint_status_vector"][1]["waypoint_pose"]["pose"]["position"]["__type"],
    "geometry_msgs::msg::Point");
  ASSERT_EQ(json["waypoint_status_vector"][1]["waypoint_pose"]["pose"]["position"]["x"], 15.0);
  ASSERT_EQ(json["waypoint_status_vector"][1]["waypoint_pose"]["pose"]["position"]["y"], 16.0);
  ASSERT_EQ(json["waypoint_status_vector"][1]["waypoint_pose"]["pose"]["position"]["z"], 17.0);
  ASSERT_EQ(
    json["waypoint_status_vector"][1]["waypoint_pose"]["pose"]["orientation"]["__type"],
    "geometry_msgs::msg::Quaternion");
  ASSERT_EQ(json["waypoint_status_vector"][1]["waypoint_pose"]["pose"]["orientation"]["x"], 18.0);
  ASSERT_EQ(json["waypoint_status_vector"][1]["waypoint_pose"]["pose"]["orientation"]["y"], 19.0);
  ASSERT_EQ(json["waypoint_status_vector"][1]["waypoint_pose"]["pose"]["orientation"]["z"], 20.0);
  ASSERT_EQ(json["waypoint_status_vector"][1]["waypoint_pose"]["pose"]["orientation"]["w"], 21.0);
  ASSERT_EQ(json["waypoint_status_vector"][1]["error_code"], 22);
  ASSERT_EQ(json["waypoint_status_vector"][1]["error_msg"], "error2");

  // Check the two-ways transform, i.e. "from_json"
  // auto waypoint_status_vector_test2 =
  //   exporter.fromJson(json["waypoint_status_vector"])->first.cast<std::vector<nav2_msgs::msg::WaypointStatus>>();
}
