// Copyright (c) 2026 Dexory
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

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/exclusion_zone.hpp"
#include "nav2_collision_monitor/polygon_utils.hpp"
#include "nav2_collision_monitor/source.hpp"

using namespace std::chrono_literals;

static constexpr double EPSILON = 1e-5;

static const char BASE_FRAME_ID[]{"base_link"};
static const char GLOBAL_FRAME_ID[]{"odom"};
static const char ZONE_FRAME_ID[]{"zone_frame"};
static const char MISSING_FRAME_ID[]{"missing_frame"};
static const char ZONE_NAME[]{"dock"};
static const char SOURCE_NAME[]{"TestSource"};
static const tf2::Duration TRANSFORM_TOLERANCE{tf2::durationFromSec(0.5)};
static const rclcpp::Duration SOURCE_TIMEOUT{rclcpp::Duration::from_seconds(5.0)};

// A square polygon (in VVF format) spanning x, y in [-1, 1].
static const char UNIT_SQUARE[]{"[[1.0, 1.0], [1.0, -1.0], [-1.0, -1.0], [-1.0, 1.0]]"};

// ---------------------------------------------------------------------------
// polygon_utils free-function tests (no node needed)
// ---------------------------------------------------------------------------

TEST(PolygonUtilsTest, ParseValidPolygon)
{
  std::vector<nav2_collision_monitor::Point> out;
  std::string error;
  ASSERT_TRUE(nav2_collision_monitor::parsePolygonPoints(UNIT_SQUARE, 3, out, error));
  EXPECT_TRUE(error.empty());
  ASSERT_EQ(out.size(), 4u);
  EXPECT_NEAR(out[0].x, 1.0, EPSILON);
  EXPECT_NEAR(out[0].y, 1.0, EPSILON);
  EXPECT_NEAR(out[2].x, -1.0, EPSILON);
  EXPECT_NEAR(out[2].y, -1.0, EPSILON);
}

TEST(PolygonUtilsTest, ParseRejectsTooFewVertices)
{
  std::vector<nav2_collision_monitor::Point> out;
  std::string error;
  // Only 2 vertices, but 3 required.
  EXPECT_FALSE(
    nav2_collision_monitor::parsePolygonPoints("[[0.0, 0.0], [1.0, 1.0]]", 3, out, error));
  EXPECT_FALSE(error.empty());
}

TEST(PolygonUtilsTest, ParseRejectsNonPairPoint)
{
  std::vector<nav2_collision_monitor::Point> out;
  std::string error;
  // Middle point has three components.
  EXPECT_FALSE(
    nav2_collision_monitor::parsePolygonPoints(
      "[[0.0, 0.0], [1.0, 1.0, 1.0], [2.0, 2.0]]", 3, out, error));
  EXPECT_FALSE(error.empty());
}

TEST(PolygonUtilsTest, ParseRejectsMalformedString)
{
  std::vector<nav2_collision_monitor::Point> out;
  std::string error;
  EXPECT_FALSE(nav2_collision_monitor::parsePolygonPoints("[[1.0, 2.0], ", 3, out, error));
  EXPECT_FALSE(error.empty());
}

TEST(PolygonUtilsTest, TransformIdentityLeavesPointsUnchanged)
{
  std::vector<nav2_collision_monitor::Point> in{{1.0, 2.0, 0.0, ""}, {-3.0, 4.0, 0.0, ""}};
  std::vector<nav2_collision_monitor::Point> out;
  tf2::Transform identity;
  identity.setIdentity();
  nav2_collision_monitor::transformPolygonPoints(identity, in, out);
  ASSERT_EQ(out.size(), in.size());
  EXPECT_NEAR(out[0].x, 1.0, EPSILON);
  EXPECT_NEAR(out[0].y, 2.0, EPSILON);
  EXPECT_NEAR(out[1].x, -3.0, EPSILON);
  EXPECT_NEAR(out[1].y, 4.0, EPSILON);
}

TEST(PolygonUtilsTest, TransformAppliesTranslation)
{
  std::vector<nav2_collision_monitor::Point> in{{1.0, 1.0, 0.0, ""}};
  std::vector<nav2_collision_monitor::Point> out;
  tf2::Transform tf;
  tf.setIdentity();
  tf.setOrigin(tf2::Vector3(10.0, -5.0, 0.0));
  nav2_collision_monitor::transformPolygonPoints(tf, in, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].x, 11.0, EPSILON);
  EXPECT_NEAR(out[0].y, -4.0, EPSILON);
}

TEST(PolygonUtilsTest, CircleToPolygonHasCorrectVertices)
{
  const double radius = 2.0;
  const int edges = 16;
  const std::vector<nav2_collision_monitor::Point> poly =
    nav2_collision_monitor::circleToPolygon(radius, edges);
  ASSERT_EQ(poly.size(), static_cast<std::size_t>(edges));
  // Every vertex must lie on the circle of the given radius.
  for (const auto & p : poly) {
    EXPECT_NEAR(std::hypot(p.x, p.y), radius, EPSILON);
  }
  // First vertex is at angle 0.
  EXPECT_NEAR(poly[0].x, radius, EPSILON);
  EXPECT_NEAR(poly[0].y, 0.0, EPSILON);
}

// ---------------------------------------------------------------------------
// ExclusionZone tests (require a node + TF)
// ---------------------------------------------------------------------------

class TestNode : public nav2::LifecycleNode
{
public:
  TestNode()
  : nav2::LifecycleNode("exclusion_zone_test_node")
  {
  }
};  // TestNode

// Minimal concrete Source that appends a fixed set of points, used to exercise
// Source::getData()'s exclusion-zone masking of newly-added points.
class FakeSource : public nav2_collision_monitor::Source
{
public:
  FakeSource(
    const nav2::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id)
  : nav2_collision_monitor::Source(
      node, source_name, tf_buffer, base_frame_id, base_frame_id,
      TRANSFORM_TOLERANCE, SOURCE_TIMEOUT, false)
  {
  }

  // Expose the protected zone configuration for testing.
  bool setupExclusionZones()
  {
    return configure();
  }

  // Points appended by the next getData() call.
  std::vector<nav2_collision_monitor::Point> points_to_add;

protected:
  bool getSourceData(
    const rclcpp::Time & /*curr_time*/,
    std::vector<nav2_collision_monitor::Point> & data) override
  {
    data.insert(data.end(), points_to_add.begin(), points_to_add.end());
    return true;
  }
};  // FakeSource

class ExclusionZoneTester : public ::testing::Test
{
public:
  ExclusionZoneTester()
  {
    node_ = std::make_shared<TestNode>();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
  }

  ~ExclusionZoneTester() override
  {
    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    node_.reset();
  }

protected:
  // Broadcast a base_frame -> child transform and wait for it to be available.
  // Also publishes an identity odom -> base_link leg, because the zone resolves
  // its (possibly held) pose through the global frame, so that leg must exist.
  void broadcastTransform(
    const std::string & child_frame, double tx, double ty)
  {
    const rclcpp::Time stamp = node_->now();

    geometry_msgs::msg::TransformStamped odom_msg;
    odom_msg.header.frame_id = GLOBAL_FRAME_ID;
    odom_msg.child_frame_id = BASE_FRAME_ID;
    odom_msg.header.stamp = stamp;
    odom_msg.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(odom_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = BASE_FRAME_ID;
    tf_msg.child_frame_id = child_frame;
    tf_msg.header.stamp = stamp;
    tf_msg.transform.translation.x = tx;
    tf_msg.transform.translation.y = ty;
    tf_msg.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_msg);

    // Wait for both legs to propagate into the buffer.
    rclcpp::Time start = node_->now();
    while (rclcpp::ok() && (node_->now() - start) < rclcpp::Duration::from_seconds(5.0)) {
      if (tf_buffer_->canTransform(child_frame, BASE_FRAME_ID, tf2::TimePointZero) &&
        tf_buffer_->canTransform(BASE_FRAME_ID, GLOBAL_FRAME_ID, tf2::TimePointZero))
      {
        return;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(10ms);
    }
    FAIL() << "Transforms for " << child_frame << " never became available";
  }

  // Spin until the latest odom -> base_link transform reports the expected x of
  // base_link expressed in odom (used when a moved pose is broadcast at a newer
  // stamp and we must wait for it to become the latest available transform).
  void waitForLatestBaseInOdomX(double expected_x)
  {
    rclcpp::Time start = node_->now();
    while (rclcpp::ok() && (node_->now() - start) < rclcpp::Duration::from_seconds(5.0)) {
      try {
        const auto t =
          tf_buffer_->lookupTransform(GLOBAL_FRAME_ID, BASE_FRAME_ID, tf2::TimePointZero);
        if (std::abs(t.transform.translation.x - expected_x) < 1e-6) {
          return;
        }
      } catch (const tf2::TransformException &) {}
      executor_->spin_some();
      std::this_thread::sleep_for(10ms);
    }
    FAIL() << "Latest odom -> base_link never reached x = " << expected_x;
  }

  // Broadcast an arbitrary parent -> child transform and wait for it to be available.
  void broadcastFrame(
    const std::string & parent_frame, const std::string & child_frame, double tx, double ty,
    const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.header.stamp = stamp;
    tf_msg.transform.translation.x = tx;
    tf_msg.transform.translation.y = ty;
    tf_msg.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_msg);

    rclcpp::Time start = node_->now();
    while (rclcpp::ok() && (node_->now() - start) < rclcpp::Duration::from_seconds(5.0)) {
      if (tf_buffer_->canTransform(child_frame, parent_frame, tf2::TimePointZero)) {
        return;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(10ms);
    }
    FAIL() << "Transform " << parent_frame << " -> " << child_frame << " never became available";
  }

  // Declare the parameters describing a zone before it is configured.
  void declareZoneParams(
    const std::string & name, const std::string & type, bool enabled,
    const std::string & frame_id)
  {
    node_->declare_parameter(name + ".enabled", rclcpp::ParameterValue(enabled));
    node_->declare_parameter(name + ".type", rclcpp::ParameterValue(type));
    node_->declare_parameter(name + ".frame_id", rclcpp::ParameterValue(frame_id));
  }

  std::shared_ptr<nav2_collision_monitor::ExclusionZone> makeZone(
    bool base_shift_correction = false)
  {
    return std::make_shared<nav2_collision_monitor::ExclusionZone>(
      node_, ZONE_NAME, tf_buffer_, BASE_FRAME_ID, GLOBAL_FRAME_ID,
      TRANSFORM_TOLERANCE, base_shift_correction);
  }

  std::shared_ptr<TestNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};  // ExclusionZoneTester

TEST_F(ExclusionZoneTester, DisabledZoneKeepsAllPoints)
{
  declareZoneParams(ZONE_NAME, "polygon", false, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  broadcastTransform(ZONE_FRAME_ID, 0.0, 0.0);

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());
  EXPECT_FALSE(zone->getEnabled());
  EXPECT_EQ(zone->getName(), std::string(ZONE_NAME));

  std::vector<nav2_collision_monitor::Point> data{{0.0, 0.0, 0.0, ""}, {5.0, 5.0, 0.0, ""}};
  zone->apply(node_->now(), data);
  // Disabled -> nothing removed even though (0, 0) is inside.
  EXPECT_EQ(data.size(), 2u);
}

TEST_F(ExclusionZoneTester, PolygonZoneMasksInsideKeepsOutside)
{
  declareZoneParams(ZONE_NAME, "polygon", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  broadcastTransform(ZONE_FRAME_ID, 0.0, 0.0);

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());
  EXPECT_TRUE(zone->getEnabled());

  std::vector<nav2_collision_monitor::Point> data{
    {0.0, 0.0, 0.0, ""},    // inside -> removed
    {0.5, -0.5, 0.0, ""},   // inside -> removed
    {2.0, 2.0, 0.0, ""},    // outside -> kept
    {-3.0, 0.0, 0.0, ""}};  // outside -> kept
  zone->apply(node_->now(), data);
  ASSERT_EQ(data.size(), 2u);
  for (const auto & p : data) {
    EXPECT_GT(std::abs(p.x), 1.0);
  }
}

TEST_F(ExclusionZoneTester, MaskTracksZoneFrameTranslation)
{
  declareZoneParams(ZONE_NAME, "polygon", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  // Shift the zone frame by (10, 0): the mask now covers base-frame x in [9, 11].
  broadcastTransform(ZONE_FRAME_ID, 10.0, 0.0);

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());

  std::vector<nav2_collision_monitor::Point> data{
    {0.0, 0.0, 0.0, ""},     // was inside at origin, now outside -> kept
    {10.0, 0.0, 0.0, ""}};   // inside the shifted zone -> removed
  zone->apply(node_->now(), data);
  ASSERT_EQ(data.size(), 1u);
  EXPECT_NEAR(data[0].x, 0.0, EPSILON);
}

TEST_F(ExclusionZoneTester, BaseShiftCorrectionMasksViaGlobalFrame)
{
  declareZoneParams(ZONE_NAME, "polygon", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  // With base-shift correction the zone lookup is bridged through the global
  // frame, so the TF tree must connect zone_frame and base_link via odom.
  // Broadcast and evaluate at a single stamp so the time-aware lookup matches exactly.
  const rclcpp::Time stamp = node_->now();
  broadcastFrame(GLOBAL_FRAME_ID, BASE_FRAME_ID, 0.0, 0.0, stamp);
  broadcastFrame(BASE_FRAME_ID, ZONE_FRAME_ID, 0.0, 0.0, stamp);

  auto zone = makeZone(/*base_shift_correction=*/true);
  ASSERT_TRUE(zone->configure());

  std::vector<nav2_collision_monitor::Point> data{
    {0.0, 0.0, 0.0, ""},    // inside -> removed
    {2.0, 2.0, 0.0, ""}};   // outside -> kept
  zone->apply(stamp, data);
  ASSERT_EQ(data.size(), 1u);
  EXPECT_NEAR(data[0].x, 2.0, EPSILON);
}


TEST_F(ExclusionZoneTester, HeightBandLimitsMasking)
{
  declareZoneParams(ZONE_NAME, "polygon", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  node_->declare_parameter(std::string(ZONE_NAME) + ".min_height", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(std::string(ZONE_NAME) + ".max_height", rclcpp::ParameterValue(0.5));
  broadcastTransform(ZONE_FRAME_ID, 0.0, 0.0);

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());

  std::vector<nav2_collision_monitor::Point> data{
    {0.0, 0.0, 0.2, ""},    // inside polygon, within height band -> removed
    {0.0, 0.0, 0.9, ""},    // inside polygon, above band -> kept
    {0.0, 0.0, -0.1, ""}};  // inside polygon, below band -> kept
  zone->apply(node_->now(), data);
  ASSERT_EQ(data.size(), 2u);
  for (const auto & p : data) {
    EXPECT_TRUE(p.z > 0.5 || p.z < 0.0);
  }
}

TEST_F(ExclusionZoneTester, CircleZoneMasksWithinRadius)
{
  declareZoneParams(ZONE_NAME, "circle", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".radius", rclcpp::ParameterValue(1.0));
  broadcastTransform(ZONE_FRAME_ID, 0.0, 0.0);

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());

  std::vector<nav2_collision_monitor::Point> data{
    {0.0, 0.0, 0.0, ""},    // centre -> removed
    {0.5, 0.5, 0.0, ""},    // dist ~0.71 < 1 -> removed
    {1.5, 0.0, 0.0, ""}};   // dist 1.5 > 1 -> kept
  zone->apply(node_->now(), data);
  ASSERT_EQ(data.size(), 1u);
  EXPECT_NEAR(data[0].x, 1.5, EPSILON);
}

TEST_F(ExclusionZoneTester, FailSafeKeepsPointsWhenTransformMissing)
{
  declareZoneParams(ZONE_NAME, "polygon", true, MISSING_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  // Deliberately do NOT broadcast MISSING_FRAME_ID.

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());

  std::vector<nav2_collision_monitor::Point> data{{0.0, 0.0, 0.0, ""}, {0.5, 0.5, 0.0, ""}};
  zone->apply(node_->now(), data);
  // No transform -> fail-safe -> nothing removed.
  EXPECT_EQ(data.size(), 2u);
}

TEST_F(ExclusionZoneTester, DynamicEnableToggle)
{
  declareZoneParams(ZONE_NAME, "polygon", false, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  broadcastTransform(ZONE_FRAME_ID, 0.0, 0.0);

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());
  EXPECT_FALSE(zone->getEnabled());

  node_->set_parameter(rclcpp::Parameter(std::string(ZONE_NAME) + ".enabled", true));
  EXPECT_TRUE(zone->getEnabled());

  std::vector<nav2_collision_monitor::Point> data{{0.0, 0.0, 0.0, ""}, {5.0, 5.0, 0.0, ""}};
  zone->apply(node_->now(), data);
  ASSERT_EQ(data.size(), 1u);
  EXPECT_NEAR(data[0].x, 5.0, EPSILON);
}

TEST_F(ExclusionZoneTester, DynamicPointsUpdateChangesMask)
{
  declareZoneParams(ZONE_NAME, "polygon", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  broadcastTransform(ZONE_FRAME_ID, 0.0, 0.0);

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());

  // A point at (2, 2) is outside the unit square.
  {
    std::vector<nav2_collision_monitor::Point> data{{2.0, 2.0, 0.0, ""}};
    zone->apply(node_->now(), data);
    EXPECT_EQ(data.size(), 1u);
  }

  // Grow the polygon to span [-3, 3]: (2, 2) now falls inside and is masked.
  const auto result = node_->set_parameter(
    rclcpp::Parameter(
      std::string(ZONE_NAME) + ".points",
      "[[3.0, 3.0], [3.0, -3.0], [-3.0, -3.0], [-3.0, 3.0]]"));
  EXPECT_TRUE(result.successful);

  {
    std::vector<nav2_collision_monitor::Point> data{{2.0, 2.0, 0.0, ""}};
    zone->apply(node_->now(), data);
    EXPECT_EQ(data.size(), 0u);
  }
}

TEST_F(ExclusionZoneTester, DynamicPointsUpdateRejectsInvalidPolygon)
{
  declareZoneParams(ZONE_NAME, "polygon", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  broadcastTransform(ZONE_FRAME_ID, 0.0, 0.0);

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());

  // Fewer than three vertices -> rejected, live polygon left unchanged.
  const auto result = node_->set_parameter(
    rclcpp::Parameter(std::string(ZONE_NAME) + ".points", "[[0.0, 0.0], [1.0, 1.0]]"));
  EXPECT_FALSE(result.successful);

  // Original unit square still masks the origin.
  std::vector<nav2_collision_monitor::Point> data{{0.0, 0.0, 0.0, ""}, {5.0, 5.0, 0.0, ""}};
  zone->apply(node_->now(), data);
  ASSERT_EQ(data.size(), 1u);
  EXPECT_NEAR(data[0].x, 5.0, EPSILON);
}

TEST_F(ExclusionZoneTester, ConfigureFailsOnInvalidPolygon)
{
  declareZoneParams(ZONE_NAME, "polygon", true, ZONE_FRAME_ID);
  // Only two vertices -> invalid.
  node_->declare_parameter(
    std::string(ZONE_NAME) + ".points",
    rclcpp::ParameterValue("[[0.0, 0.0], [1.0, 1.0]]"));

  auto zone = makeZone();
  EXPECT_FALSE(zone->configure());
}

TEST_F(ExclusionZoneTester, ConfigureFailsOnInvalidCircleRadius)
{
  declareZoneParams(ZONE_NAME, "circle", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".radius", rclcpp::ParameterValue(-1.0));

  auto zone = makeZone();
  EXPECT_FALSE(zone->configure());
}

TEST_F(ExclusionZoneTester, ConfigureFailsOnUnknownType)
{
  declareZoneParams(ZONE_NAME, "triangle", true, ZONE_FRAME_ID);

  auto zone = makeZone();
  EXPECT_FALSE(zone->configure());
}

// ---------------------------------------------------------------------------
// Flaky-frame hold behaviour
// ---------------------------------------------------------------------------

TEST_F(ExclusionZoneTester, HeldZoneKeepsMaskingWithinWindow)
{
  declareZoneParams(ZONE_NAME, "polygon", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  node_->declare_parameter(
    std::string(ZONE_NAME) + ".frame_hold_timeout", rclcpp::ParameterValue(5.0));

  // Detection at stamp0; robot at the odom origin, zone at the origin.
  const rclcpp::Time stamp0 = node_->now();
  broadcastFrame(GLOBAL_FRAME_ID, BASE_FRAME_ID, 0.0, 0.0, stamp0);
  broadcastFrame(BASE_FRAME_ID, ZONE_FRAME_ID, 0.0, 0.0, stamp0);

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());

  // 2 s later the frame has not refreshed, but that is within the 5 s hold
  // window, so the zone must keep masking at its last known pose.
  const rclcpp::Time later = stamp0 + rclcpp::Duration::from_seconds(2.0);
  std::vector<nav2_collision_monitor::Point> data{{0.0, 0.0, 0.0, ""}, {5.0, 5.0, 0.0, ""}};
  zone->apply(later, data);
  ASSERT_EQ(data.size(), 1u);
  EXPECT_NEAR(data[0].x, 5.0, EPSILON);
}

TEST_F(ExclusionZoneTester, HeldZoneFailsSafeAfterWindowExpires)
{
  declareZoneParams(ZONE_NAME, "polygon", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  node_->declare_parameter(
    std::string(ZONE_NAME) + ".frame_hold_timeout", rclcpp::ParameterValue(1.0));

  const rclcpp::Time stamp0 = node_->now();
  broadcastFrame(GLOBAL_FRAME_ID, BASE_FRAME_ID, 0.0, 0.0, stamp0);
  broadcastFrame(BASE_FRAME_ID, ZONE_FRAME_ID, 0.0, 0.0, stamp0);

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());

  // 3 s later exceeds the 1 s hold window -> fail safe, keep all points.
  const rclcpp::Time later = stamp0 + rclcpp::Duration::from_seconds(3.0);
  std::vector<nav2_collision_monitor::Point> data{{0.0, 0.0, 0.0, ""}, {5.0, 5.0, 0.0, ""}};
  zone->apply(later, data);
  EXPECT_EQ(data.size(), 2u);
}

TEST_F(ExclusionZoneTester, HeldZoneStaysFixedInWorldAsRobotMoves)
{
  declareZoneParams(ZONE_NAME, "polygon", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  node_->declare_parameter(
    std::string(ZONE_NAME) + ".frame_hold_timeout", rclcpp::ParameterValue(5.0));

  // Robot at the odom origin; charger detected 10 m ahead -> world pose (10, 0).
  const rclcpp::Time stamp0 = node_->now();
  broadcastFrame(GLOBAL_FRAME_ID, BASE_FRAME_ID, 0.0, 0.0, stamp0);
  broadcastFrame(BASE_FRAME_ID, ZONE_FRAME_ID, 10.0, 0.0, stamp0);

  // Robot drives +5 m in x; the detection is NOT refreshed (zone frozen at stamp0).
  const rclcpp::Time stamp1 = stamp0 + rclcpp::Duration::from_seconds(0.5);
  broadcastFrame(GLOBAL_FRAME_ID, BASE_FRAME_ID, 5.0, 0.0, stamp1);
  waitForLatestBaseInOdomX(5.0);

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());

  // The held pose stays at world (10, 0); re-projected into the base moved to
  // (5, 0) it now sits at base-frame x = 5. A buggy robot-attached hold would
  // instead keep the mask at base-frame x = 10.
  std::vector<nav2_collision_monitor::Point> data{
    {5.0, 0.0, 0.0, ""},     // charger's world spot in the current base -> removed
    {10.0, 0.0, 0.0, ""}};   // where a robot-attached hold would sit -> kept
  zone->apply(stamp1, data);
  ASSERT_EQ(data.size(), 1u);
  EXPECT_NEAR(data[0].x, 10.0, EPSILON);
}

// ---------------------------------------------------------------------------
// Visualization
// ---------------------------------------------------------------------------

TEST_F(ExclusionZoneTester, VisualizationPublishesMaskInBaseFrame)
{
  declareZoneParams(ZONE_NAME, "polygon", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  node_->declare_parameter(std::string(ZONE_NAME) + ".visualize", rclcpp::ParameterValue(true));
  broadcastTransform(ZONE_FRAME_ID, 0.0, 0.0);

  geometry_msgs::msg::PolygonStamped::ConstSharedPtr received;
  auto sub = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
    std::string("~/") + ZONE_NAME,
    [&](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {received = msg;},
    nav2::qos::StandardTopicQoS());

  auto zone = makeZone();
  ASSERT_TRUE(zone->configure());
  zone->activate();

  // Publishing is decoupled from the flaky zone frame: the polygon must be
  // emitted in the smooth base frame so a consumer never depends on that frame.
  rclcpp::Time start = node_->now();
  while (rclcpp::ok() && !received &&
    (node_->now() - start) < rclcpp::Duration::from_seconds(5.0))
  {
    zone->publish();
    executor_->spin_some();
    std::this_thread::sleep_for(10ms);
  }
  zone->deactivate();

  ASSERT_NE(received, nullptr);
  EXPECT_EQ(received->header.frame_id, std::string(BASE_FRAME_ID));
  EXPECT_EQ(received->polygon.points.size(), 4u);
}

// ---------------------------------------------------------------------------
// Source::getData() masking integration
// ---------------------------------------------------------------------------

TEST_F(ExclusionZoneTester, SourceMasksOnlyNewlyAddedPoints)
{
  declareZoneParams(ZONE_NAME, "polygon", true, ZONE_FRAME_ID);
  node_->declare_parameter(std::string(ZONE_NAME) + ".points", rclcpp::ParameterValue(UNIT_SQUARE));
  node_->declare_parameter(
    std::string(SOURCE_NAME) + ".exclusion_zones",
    rclcpp::ParameterValue(std::vector<std::string>{ZONE_NAME}));
  broadcastTransform(ZONE_FRAME_ID, 0.0, 0.0);

  auto source = std::make_shared<FakeSource>(node_, SOURCE_NAME, tf_buffer_, BASE_FRAME_ID);
  ASSERT_TRUE(source->setupExclusionZones());

  // The source will append one point inside the zone and one outside.
  source->points_to_add = {{0.0, 0.0, 0.0, ""}, {5.0, 5.0, 0.0, ""}};

  // Pre-existing point already inside the zone must be preserved (it does not
  // belong to this source's new data).
  std::vector<nav2_collision_monitor::Point> data{{0.2, 0.2, 0.0, ""}};
  ASSERT_TRUE(source->getData(node_->now(), data));

  // Expect: pre-existing inside point kept, new inside point removed, new
  // outside point kept.
  ASSERT_EQ(data.size(), 2u);
  EXPECT_NEAR(data[0].x, 0.2, EPSILON);
  EXPECT_NEAR(data[1].x, 5.0, EPSILON);
}

TEST_F(ExclusionZoneTester, SourceWithoutZonesLeavesDataUnchanged)
{
  node_->declare_parameter(
    std::string(SOURCE_NAME) + ".exclusion_zones",
    rclcpp::ParameterValue(std::vector<std::string>{}));

  auto source = std::make_shared<FakeSource>(node_, SOURCE_NAME, tf_buffer_, BASE_FRAME_ID);
  ASSERT_TRUE(source->setupExclusionZones());

  source->points_to_add = {{0.0, 0.0, 0.0, ""}, {5.0, 5.0, 0.0, ""}};
  std::vector<nav2_collision_monitor::Point> data;
  ASSERT_TRUE(source->getData(node_->now(), data));
  EXPECT_EQ(data.size(), 2u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
