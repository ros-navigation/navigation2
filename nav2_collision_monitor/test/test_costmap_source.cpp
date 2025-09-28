#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_collision_monitor/costmap.hpp"

// Helper: build a tiny costmap with exactly one lethal cell.
// Frame is "base_link" so TF transform is identity (keeps the test simple).
static nav2_msgs::msg::Costmap makeCostmapMsg(
  const std::string & frame_id,
  uint32_t w, uint32_t h, float res,
  uint32_t lethal_x, uint32_t lethal_y)
{
  nav2_msgs::msg::Costmap m;
  m.header.stamp = rclcpp::Clock().now();           // fresh timestamp (freshness check passes)
  m.header.frame_id = frame_id;                     // same as base_frame_id => identity TF
  m.metadata.size_x = w;
  m.metadata.size_y = h;
  m.metadata.resolution = res;
  m.metadata.origin.position.x = 0.0;
  m.metadata.origin.position.y = 0.0;
  m.data.assign(w * h, 0);
  m.data[lethal_y * w + lethal_x] = 254;           // mark one lethal cell
  return m;
}

TEST(CostmapSource, LethalCellProducesExactlyOnePoint)
{
  // Node / TF setup
  auto node = std::make_shared<nav2::LifecycleNode>("cm_test_node");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Construct the class under test (regular class, not pluginlib)
  nav2_collision_monitor::CostmapSource src(
    node,                   // Lifecycle node
    "grid",                 // source_name (param prefix)
    tf_buffer,              // TF buffer
    "base_link",            // base_frame_id
    "base_link",            // global_frame_id (unused here)
    tf2::durationFromSec(0.1),
    rclcpp::Duration::from_seconds(1.0), // source_timeout
    false);

  // Declare parameters the source expects
  node->declare_parameter("grid.topic", "/test_costmap");
  node->declare_parameter("grid.enabled", true);
  node->declare_parameter("grid.cost_threshold", 254);
  node->declare_parameter("grid.source_timeout", 1.0);

  // Configure (creates the subscription)
  src.configure();

  // Publisher + executor (to drive callbacks)
  auto pub_node = std::make_shared<rclcpp::Node>("pub_node_costmap_ok");         // <-- plain node
  auto pub = pub_node->create_publisher<nav2_msgs::msg::Costmap>("/test_costmap", 10);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());  // SMAP/CostmapSource node
  exec.add_node(pub_node);                         // publisher node

  // Publish a 3x3 grid with one lethal cell at (x=1, y=0)
  auto msg = makeCostmapMsg("base_link", 3, 3, 0.1f, 1, 0);
  msg.header.stamp = node->now();                  // align with SMAP node clock
  pub->publish(msg);

  // Pump callbacks until getData() returns points or timeout
  const auto start = node->now();
  const auto deadline = rclcpp::Duration::from_seconds(1.0);
  while (rclcpp::ok() && (node->now() - start) < deadline) {
    exec.spin_some();
    std::vector<nav2_collision_monitor::Point> pts;
    if (src.getData(node->now(), pts)) {
      // Exactly one obstacle expected
      ASSERT_EQ(pts.size(), 1u);

      // The cell center for (1,0) with resolution 0.1:
      // x = (1 + 0.5) * 0.1 = 0.15, y = (0 + 0.5) * 0.1 = 0.05
      EXPECT_NEAR(pts[0].x, 0.15, 1e-3);
      EXPECT_NEAR(pts[0].y, 0.05, 1e-3);
      return;  // success
    }
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
  FAIL() << "CostmapSource did not produce points in time";
}

TEST(CostmapSource, StaleMessageIsIgnored)
{
  auto node = std::make_shared<nav2::LifecycleNode>("cm_test_node2");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  nav2_collision_monitor::CostmapSource src(
    node, "grid", tf_buffer, "base_link", "base_link",
    tf2::durationFromSec(0.1),
    rclcpp::Duration::from_seconds(0.01),  // very short timeout
    false);

  node->declare_parameter("grid.topic", "/test_costmap2");
  node->declare_parameter("grid.enabled", true);
  node->declare_parameter("grid.cost_threshold", 254);
  node->declare_parameter("grid.source_timeout", 0.01);

  src.configure();

  auto pub_node = std::make_shared<rclcpp::Node>("pub_node_costmap_stale");      // <-- plain node
  auto pub = pub_node->create_publisher<nav2_msgs::msg::Costmap>("/test_costmap2", 10);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(pub_node);

  // Publish a stale message (timestamp way in the past)
  auto msg = makeCostmapMsg("base_link", 2, 2, 0.1f, 0, 0);
  msg.header.stamp = node->now() - rclcpp::Duration::from_seconds(10.0);
  pub->publish(msg);

  // Spin once to deliver the stale message, then ensure getData() rejects it
  exec.spin_some();
  std::vector<nav2_collision_monitor::Point> pts;
  EXPECT_FALSE(src.getData(node->now(), pts));  // should be ignored due to freshness check
}

TEST(CostmapSource, RespectsCostThreshold)
{
  // Single publisher node shared by both runs (plain rclcpp node; no lifecycle)
  auto pub_node = std::make_shared<rclcpp::Node>("pub_node_costmap_thresh");
  auto pub = pub_node->create_publisher<nav2_msgs::msg::Costmap>("/th_costmap", 10);

  // Helper to publish the same 3x1 costmap: [254, 253, 0]
  auto make_msg = [](const rclcpp::Time & stamp) {
    nav2_msgs::msg::Costmap m = makeCostmapMsg("base_link", 3, 1, 0.1f, 0, 0);
    m.data[0] = 254; // lethal
    m.data[1] = 253; // inscribed
    m.data[2] = 0;
    m.header.stamp = stamp;
    return m;
  };

  // ---------- Run A: threshold = 254 -> expect 1 point ----------
  {
    auto node = std::make_shared<nav2::LifecycleNode>("cm_thresh_node_A");
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Set params BEFORE configure()
    node->declare_parameter("grid.topic", "/th_costmap");
    node->declare_parameter("grid.enabled", true);
    node->declare_parameter("grid.source_timeout", 1.0);
    node->declare_parameter("grid.cost_threshold", 254);

    nav2_collision_monitor::CostmapSource src(
      node, "grid", tf_buffer,
      "base_link", "base_link",
      tf2::durationFromSec(0.05),
      rclcpp::Duration::from_seconds(1.0),
      false);

    src.configure();

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());

    auto msg = make_msg(node->now());
    pub->publish(msg);

    auto start = node->now();
    bool ok = false;
    while ((node->now() - start) < rclcpp::Duration::from_seconds(1.0)) {
      exec.spin_some();
      std::vector<nav2_collision_monitor::Point> pts;
      if (src.getData(node->now(), pts)) {
        ASSERT_EQ(pts.size(), 1u);
        ok = true;
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(ok) << "threshold=254 case didn’t produce points";
  }

  // ---------- Run B: threshold = 253 -> expect 2 points ----------
  {
    auto node = std::make_shared<nav2::LifecycleNode>("cm_thresh_node_B");
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Set params BEFORE configure()
    node->declare_parameter("grid.topic", "/th_costmap");
    node->declare_parameter("grid.enabled", true);
    node->declare_parameter("grid.source_timeout", 1.0);
    node->declare_parameter("grid.cost_threshold", 253);

    nav2_collision_monitor::CostmapSource src(
      node, "grid", tf_buffer,
      "base_link", "base_link",
      tf2::durationFromSec(0.05),
      rclcpp::Duration::from_seconds(1.0),
      false);

    src.configure();

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());

    auto msg = make_msg(node->now());
    pub->publish(msg);

    auto start = node->now();
    bool ok = false;
    while ((node->now() - start) < rclcpp::Duration::from_seconds(1.0)) {
      exec.spin_some();
      std::vector<nav2_collision_monitor::Point> pts;
      if (src.getData(node->now(), pts)) {
        ASSERT_EQ(pts.size(), 2u);
        ok = true;
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(ok) << "threshold=253 case didn’t produce points";
  }
}


#include <geometry_msgs/msg/transform_stamped.hpp>

TEST(CostmapSource, TransformsFromGlobalToBase)
{
  auto node = std::make_shared<nav2::LifecycleNode>("cm_tf_node");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Inject a static transform: map -> base_link is a +0.5m x-translation
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node->now();
  tf.header.frame_id = "map";
  tf.child_frame_id = "base_link";
  tf.transform.translation.x = 0.5;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer->setTransform(tf, "test_authority"));

  nav2_collision_monitor::CostmapSource src(
    node, "grid", tf_buffer,
    "base_link", "map",                   // base in base_link, message in map
    tf2::durationFromSec(0.1),
    rclcpp::Duration::from_seconds(1.0),
    false);

  node->declare_parameter("grid.topic", "/tf_costmap");
  node->declare_parameter("grid.enabled", true);
  node->declare_parameter("grid.cost_threshold", 254);
  node->declare_parameter("grid.source_timeout", 1.0);

  src.configure();

  auto pub_node = std::make_shared<rclcpp::Node>("pub_node_costmap_tf");         // <-- plain node
  auto pub = pub_node->create_publisher<nav2_msgs::msg::Costmap>("/tf_costmap", 10);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(pub_node);

  // Build a 1x1 costmap with a lethal cell at (0,0) in the MAP frame,
  // resolution 1.0, origin (0,0) => center at (0.5, 0.5) in MAP.
  nav2_msgs::msg::Costmap msg = makeCostmapMsg("map", 1, 1, 1.0f, 0, 0);
  msg.data[0] = 254;
  msg.header.stamp = node->now();                 // align with SMAP node clock
  pub->publish(msg);

  // After transform map->base_link: (0.5, 0.5) - (0.5, 0) = (0.0, 0.5)
  const auto start = node->now();
  while ((node->now() - start) < rclcpp::Duration::from_seconds(1.0)) {
    exec.spin_some();
    std::vector<nav2_collision_monitor::Point> pts;
    if (src.getData(node->now(), pts)) {
      ASSERT_EQ(pts.size(), 1u);
      EXPECT_NEAR(pts[0].x, 0.0, 1e-3);
      EXPECT_NEAR(pts[0].y, 0.5, 1e-3);
      return;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
  FAIL() << "No transformed point arrived in time";
}


// --- add this at the very end of test_costmap_source.cpp ---
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);                 // initialize ROS 2 context (fixes "context is null")
  ::testing::InitGoogleTest(&argc, argv);   // initialize gtest
  int ret = RUN_ALL_TESTS();                // run tests
  rclcpp::shutdown();                       // cleanly shutdown ROS 2
  return ret;
}
