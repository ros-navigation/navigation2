// Copyright (c) 2026 Komada (aki1770-del)
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
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/costmap_filters/zone_parameter_filter.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr char kFilterName[] = "zone_parameter_filter";
constexpr char kInfoTopic[] = "costmap_filter_info";
constexpr char kMaskTopic[] = "mask";
}  // namespace

class InfoPublisher : public rclcpp::Node
{
public:
  InfoPublisher(uint8_t type, const char * mask_topic, double base, double multiplier)
  : Node("costmap_filter_info_pub")
  {
    publisher_ = create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
      kInfoTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    auto msg = std::make_unique<nav2_msgs::msg::CostmapFilterInfo>();
    msg->type = type;
    msg->filter_mask_topic = mask_topic;
    msg->base = static_cast<float>(base);
    msg->multiplier = static_cast<float>(multiplier);
    publisher_->publish(std::move(msg));
  }

  ~InfoPublisher() override {publisher_.reset();}

private:
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;
};

class MaskPublisher : public rclcpp::Node
{
public:
  explicit MaskPublisher(const nav_msgs::msg::OccupancyGrid & mask)
  : Node("mask_pub")
  {
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      kMaskTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    publisher_->publish(mask);
  }

  ~MaskPublisher() override {publisher_.reset();}

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

class StateEventSubscriber : public rclcpp::Node
{
public:
  explicit StateEventSubscriber(const std::string & topic)
  : Node("zpf_state_sub"), last_state_(0), received_(false)
  {
    subscriber_ = create_subscription<std_msgs::msg::UInt8>(
      topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::UInt8::ConstSharedPtr msg) {
        last_state_ = msg->data;
        received_ = true;
      });
  }

  uint8_t lastState() const {return last_state_;}
  bool received() const {return received_;}

private:
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
  uint8_t last_state_;
  bool received_;
};

static nav_msgs::msg::OccupancyGrid make_mask(uint32_t w, uint32_t h, int8_t fill_value)
{
  nav_msgs::msg::OccupancyGrid mask;
  mask.header.frame_id = "map";
  mask.info.resolution = 1.0;
  mask.info.width = w;
  mask.info.height = h;
  mask.info.origin.orientation.w = 1.0;
  mask.data.assign(w * h, fill_value);
  return mask;
}

class TargetNode : public rclcpp::Node
{
public:
  TargetNode()
  : rclcpp::Node("zpf_target_node")
  {
    declare_parameter("speed", 1.0);
    declare_parameter("inflation", 0.5);
    rcl_interfaces::msg::ParameterDescriptor ro_desc;
    ro_desc.read_only = true;
    declare_parameter("readonly_speed", 1.0, ro_desc);
  }

  double getSpeed() {return get_parameter("speed").as_double();}
  double getInflation() {return get_parameter("inflation").as_double();}
};

class TestZpf : public ::testing::Test
{
protected:
  void SetUp() override
  {
    target_node_ = std::make_shared<TargetNode>();
    target_executor_.add_node(target_node_);
  }

  void TearDown() override
  {
    filter_.reset();
    info_pub_.reset();
    mask_pub_.reset();
    layers_.reset();
    if (state_event_sub_) {
      state_event_executor_.remove_node(state_event_sub_);
      state_event_sub_.reset();
    }
    if (node_) {
      node_executor_.remove_node(node_->get_node_base_interface());
    }
    node_.reset();
    target_executor_.remove_node(target_node_);
    target_node_.reset();
  }

  // Builds a ZPF host node + filter + info/mask publishers. Spins until
  // the filter becomes active or 2s pass. Optional info_type lets the
  // wrong-type test inject a non-ZPF info publisher.
  bool createFilter(
    const std::vector<int64_t> & state_ids,
    const std::vector<rclcpp::Parameter> & state_overrides,
    int8_t mask_fill_value,
    const std::string & state_event_topic = "",
    const std::vector<std::string> & target_nodes = {"zpf_target_node"},
    const std::string & on_param_set_failure = "",
    uint8_t info_type = nav2_costmap_2d::ZONE_PARAMETER_FILTER)
  {
    rclcpp::NodeOptions opts;
    std::vector<rclcpp::Parameter> all_overrides = {
      rclcpp::Parameter(std::string(kFilterName) + ".state_ids", state_ids),
      rclcpp::Parameter(std::string(kFilterName) + ".target_nodes", target_nodes),
      rclcpp::Parameter(
        std::string(kFilterName) + ".filter_info_topic",
        std::string(kInfoTopic)),
      rclcpp::Parameter(std::string(kFilterName) + ".transform_tolerance", 0.5),
    };
    if (!state_event_topic.empty()) {
      all_overrides.emplace_back(
        std::string(kFilterName) + ".state_event_topic", state_event_topic);
    }
    if (!on_param_set_failure.empty()) {
      all_overrides.emplace_back(
        std::string(kFilterName) + ".on_param_set_failure", on_param_set_failure);
    }
    for (const auto & p : state_overrides) {
      all_overrides.push_back(p);
    }
    opts.parameter_overrides(all_overrides);

    node_ = std::make_shared<nav2::LifecycleNode>("zpf_test_host", opts);
    node_executor_.add_node(node_->get_node_base_interface());

    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);

    filter_ = std::make_shared<nav2_costmap_2d::ZoneParameterFilter>();
    filter_->initialize(layers_.get(), kFilterName, tf_buffer_.get(), node_, nullptr);
    filter_->initializeFilter(kInfoTopic);

    info_pub_ = std::make_shared<InfoPublisher>(info_type, kMaskTopic, 0.0, 1.0);
    auto mask = make_mask(4, 4, mask_fill_value);
    mask_pub_ = std::make_shared<MaskPublisher>(mask);
    pub_executor_.add_node(info_pub_);
    pub_executor_.add_node(mask_pub_);

    if (!state_event_topic.empty()) {
      state_event_sub_ = std::make_shared<StateEventSubscriber>(state_event_topic);
      state_event_executor_.add_node(state_event_sub_);
    }

    auto start = node_->now();
    while (!filter_->isActive()) {
      if (node_->now() - start > rclcpp::Duration(2s)) {
        return false;
      }
      pub_executor_.spin_some();
      node_executor_.spin_some();
      target_executor_.spin_some();
      state_event_executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    return true;
  }

  void rePublishMask(int8_t fill_value)
  {
    pub_executor_.remove_node(mask_pub_);
    mask_pub_.reset();
    auto mask = make_mask(4, 4, fill_value);
    mask_pub_ = std::make_shared<MaskPublisher>(mask);
    pub_executor_.add_node(mask_pub_);
    auto start = node_->now();
    while (node_->now() - start < rclcpp::Duration(150ms)) {
      pub_executor_.spin_some();
      node_executor_.spin_some();
      target_executor_.spin_some();
      state_event_executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  void runProcess(double pose_x = 1.5, double pose_y = 1.5)
  {
    nav2_costmap_2d::Costmap2D costmap(4, 4, 1.0, 0.0, 0.0, 0);
    geometry_msgs::msg::Pose pose;
    pose.position.x = pose_x;
    pose.position.y = pose_y;
    pose.orientation.w = 1.0;
    filter_->process(costmap, 0, 0, 4, 4, pose);
  }

  template<typename Pred>
  bool waitFor(Pred pred, std::chrono::milliseconds timeout = 1500ms)
  {
    auto start = node_->now();
    while (!pred()) {
      if (node_->now() - start > rclcpp::Duration(timeout)) {
        return false;
      }
      pub_executor_.spin_some();
      node_executor_.spin_some();
      target_executor_.spin_some();
      state_event_executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    return true;
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto start = node_->now();
    while (node_->now() - start < rclcpp::Duration(duration)) {
      pub_executor_.spin_some();
      node_executor_.spin_some();
      target_executor_.spin_some();
      state_event_executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  std::shared_ptr<TargetNode> target_node_;
  std::shared_ptr<StateEventSubscriber> state_event_sub_;
  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::ZoneParameterFilter> filter_;
  std::shared_ptr<InfoPublisher> info_pub_;
  std::shared_ptr<MaskPublisher> mask_pub_;
  rclcpp::executors::SingleThreadedExecutor node_executor_;
  rclcpp::executors::SingleThreadedExecutor target_executor_;
  rclcpp::executors::SingleThreadedExecutor pub_executor_;
  rclcpp::executors::SingleThreadedExecutor state_event_executor_;
};

TEST(ZoneParameterFilter, FilterTypeConstantAndDefaultConstruction)
{
  EXPECT_NE(nav2_costmap_2d::ZONE_PARAMETER_FILTER, nav2_costmap_2d::KEEPOUT_FILTER);
  EXPECT_NE(nav2_costmap_2d::ZONE_PARAMETER_FILTER, nav2_costmap_2d::SPEED_FILTER_PERCENT);
  EXPECT_NE(nav2_costmap_2d::ZONE_PARAMETER_FILTER, nav2_costmap_2d::SPEED_FILTER_ABSOLUTE);
  EXPECT_NE(nav2_costmap_2d::ZONE_PARAMETER_FILTER, nav2_costmap_2d::BINARY_FILTER);

  nav2_costmap_2d::ZoneParameterFilter filter;
  EXPECT_FALSE(filter.isActive());
}

TEST_F(TestZpf, WrongFilterInfoTypeIsRejected)
{
  // CostmapFilterInfo with a non-ZPF type must be rejected by
  // filterInfoCallback; the mask subscription is never built and
  // isActive() never flips. createFilter() returns false on 2s timeout.
  EXPECT_FALSE(
    createFilter(
      {1},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.5),
      },
      1,
      "",
      {"zpf_target_node"},
      "",
      nav2_costmap_2d::BINARY_FILTER));
}

TEST_F(TestZpf, State1AppliesParameterAndResetFilterDeactivates)
{
  ASSERT_TRUE(createFilter(
      {1},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.5),
      },
      1)) << "Filter did not become active within 2s";

  EXPECT_DOUBLE_EQ(target_node_->getSpeed(), 1.0);

  runProcess();
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 0.5;}))
    << "Target node 'speed' did not become 0.5 within 1.5s";

  EXPECT_DOUBLE_EQ(target_node_->getInflation(), 0.5);

  filter_->resetFilter();
  EXPECT_FALSE(filter_->isActive())
    << "resetFilter() must clear filter_info_received_ and filter_mask_";
}

TEST_F(TestZpf, State0ResetsToNominalDefaults)
{
  ASSERT_TRUE(createFilter(
      {1},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.3),
    rclcpp::Parameter(
          std::string(kFilterName) + ".nominal_defaults.zpf_target_node.speed", 1.0),
      },
      1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 0.3;}));

  rePublishMask(0);
  runProcess();
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 1.0;}))
    << "speed never restored to nominal default 1.0 after state 0";
}

TEST_F(TestZpf, UnknownStateThrows)
{
  ASSERT_TRUE(createFilter(
      {1},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.3),
      },
      2)) << "Filter did not become active";

  EXPECT_THROW(runProcess(), std::runtime_error);
  EXPECT_DOUBLE_EQ(target_node_->getSpeed(), 1.0)
    << "speed must NOT change when applyState throws on unknown state";
}

TEST_F(TestZpf, StateEventPublishedOnTransition)
{
  ASSERT_TRUE(createFilter(
      {1},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.3),
      },
      1,
      "/zpf_state")) << "Filter did not become active";

  ASSERT_FALSE(state_event_sub_->received());

  runProcess();
  ASSERT_TRUE(waitFor([this]() {return state_event_sub_->received();}));
  EXPECT_EQ(state_event_sub_->lastState(), 1u);
}

TEST_F(TestZpf, ProcessHotPathReturnsPromptlyEvenWithUnreachableTargets)
{
  ASSERT_TRUE(createFilter(
      {1},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.nonexistent_node.foo", 0.5),
      },
      1,
      "",
      {"nonexistent_node"})) << "Filter did not become active";

  auto t0 = std::chrono::steady_clock::now();
  EXPECT_NO_THROW(runProcess());
  auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_LT(
    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 500);
  EXPECT_DOUBLE_EQ(target_node_->getSpeed(), 1.0);
}

TEST_F(TestZpf, UnknownMaskCellLeavesStateUnchanged)
{
  // Mask filled with OCC_GRID_UNKNOWN (-1) — process() must not transition.
  ASSERT_TRUE(createFilter(
      {1},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.5),
      },
      static_cast<int8_t>(-1))) << "Filter did not become active";

  runProcess();
  spinFor(250ms);
  EXPECT_DOUBLE_EQ(target_node_->getSpeed(), 1.0);
}

TEST_F(TestZpf, RobotOutsideMaskResetsToState0)
{
  ASSERT_TRUE(createFilter(
      {1},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.4),
    rclcpp::Parameter(
          std::string(kFilterName) + ".nominal_defaults.zpf_target_node.speed", 1.0),
      },
      1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 0.4;}));

  runProcess(100.0, 100.0);
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 1.0;}))
    << "speed never restored to nominal after out-of-mask pose";
}

TEST_F(TestZpf, LongestPrefixMatchForOverlappingTargetNodes)
{
  // target_nodes contains both "zpf" (shorter) and "zpf_target_node" (longer).
  // The state_1.zpf_target_node.speed key must route to the longer match.
  ASSERT_TRUE(createFilter(
      {1},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.7),
      },
      1,
      "",
      {"zpf", "zpf_target_node"})) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 0.7;}))
    << "speed never became 0.7 — longest-prefix-match likely broken";
}

TEST_F(TestZpf, InfoAndMaskRePublishUpdateSubscriptions)
{
  ASSERT_TRUE(createFilter(
      {1, 2},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.5),
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_2.zpf_target_node.speed", 0.2),
      },
      1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 0.5;}));

  // Re-publish info; filter must remain active and rebuild mask sub.
  pub_executor_.remove_node(info_pub_);
  info_pub_.reset();
  info_pub_ = std::make_shared<InfoPublisher>(
    nav2_costmap_2d::ZONE_PARAMETER_FILTER, kMaskTopic, 0.0, 1.0);
  pub_executor_.add_node(info_pub_);

  // Re-publish mask with state 2; same pose now samples state 2.
  rePublishMask(2);
  ASSERT_TRUE(filter_->isActive()) << "filter must remain active after info re-publish";

  runProcess();
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 0.2;}))
    << "state-2 speed never landed after mask re-publish";
}

TEST_F(TestZpf, EmptyAndInvalidConfigEdgesDoNotCrash)
{
  // Empty state_ids + empty target_nodes + invalid state_id in list +
  // unmatched override entry — all should load and operate without crashing.
  ASSERT_TRUE(createFilter(
      {0, 256, 1},  // 0 reserved, 256 > 255, 1 valid
  {
    // bogus_node not in target_nodes — warn + skip.
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.bogus_node.speed", 9.9),
    // valid entry — should apply.
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.5),
      },
      1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 0.5;}))
    << "valid override did not apply alongside invalid state_ids + unmatched node";
}

TEST_F(TestZpf, OnParamSetFailurePolicyParsing)
{
  // "warn" parses cleanly; an unrecognised value defaults to "throw" with
  // a config-load warning. Both must initializeFilter without raising.
  for (const auto & policy : std::vector<std::string>{"warn", "bogus"}) {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
        rclcpp::Parameter(std::string(kFilterName) + ".state_ids", std::vector<int64_t>{1}),
        rclcpp::Parameter(
          std::string(kFilterName) + ".target_nodes",
          std::vector<std::string>{"zpf_target_node"}),
        rclcpp::Parameter(
          std::string(kFilterName) + ".filter_info_topic", std::string(kInfoTopic)),
        rclcpp::Parameter(std::string(kFilterName) + ".transform_tolerance", 0.5),
        rclcpp::Parameter(
          std::string(kFilterName) + ".on_param_set_failure", policy),
        rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.5),
      });
    auto node = std::make_shared<nav2::LifecycleNode>("zpf_policy_test_host", opts);
    auto layers = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_buffer->setUsingDedicatedThread(true);
    auto filter = std::make_shared<nav2_costmap_2d::ZoneParameterFilter>();
    filter->initialize(layers.get(), kFilterName, tf_buffer.get(), node, nullptr);
    EXPECT_NO_THROW(filter->initializeFilter(kInfoTopic)) << "policy=" << policy;
  }
}

TEST_F(TestZpf, OnParamSetFailureThrowsByDefaultAndWarnSwallows)
{
  // Routes state_1 to TargetNode's read-only parameter; set_parameters
  // returns successful=false; checkPendingParameterUpdates applies the
  // policy. Default (throw) propagates; "warn" swallows.
  ASSERT_TRUE(createFilter(
      {1},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.readonly_speed", 0.5),
      },
      1)) << "Filter did not become active under default policy";

  EXPECT_THROW(
  {
    runProcess();
    spinFor(300ms);  // drain pending futures; failure surfaces here under kThrow
    runProcess();
  },
    std::runtime_error);

  // Tear down and rebuild with "warn" policy.
  filter_.reset();
  info_pub_.reset();
  mask_pub_.reset();
  layers_.reset();
  node_executor_.remove_node(node_->get_node_base_interface());
  node_.reset();

  ASSERT_TRUE(createFilter(
      {1},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.readonly_speed", 0.5),
      },
      1,
      "",
      {"zpf_target_node"},
      "warn")) << "Filter did not become active under warn policy";

  EXPECT_NO_THROW(
  {
    runProcess();
    spinFor(300ms);
    runProcess();
  });
}

TEST_F(TestZpf, CrossStateTransitionResetsParamTouchedByNOnly)
{
  // state_1 sets both speed AND inflation; state_2 sets speed only.
  // Transition 1→2 must restore inflation to its nominal_defaults value.
  // The in-both param (speed) must land at state_2's value, not the nominal.
  ASSERT_TRUE(createFilter(
      {1, 2},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.3),
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.inflation", 0.8),
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_2.zpf_target_node.speed", 0.5),
    rclcpp::Parameter(
          std::string(kFilterName) + ".nominal_defaults.zpf_target_node.speed", 1.0),
    rclcpp::Parameter(
          std::string(kFilterName) + ".nominal_defaults.zpf_target_node.inflation", 2.0),
      },
      1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 0.3;}));
  ASSERT_TRUE(waitFor([this]() {return target_node_->getInflation() == 0.8;}));

  rePublishMask(2);
  runProcess();
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 0.5;}))
    << "in-both param must land at state-2's value, not state-1's nominal";
  ASSERT_TRUE(waitFor([this]() {return target_node_->getInflation() == 2.0;}))
    << "inflation must be reset to nominal_defaults on 1→2 transition";
}

TEST_F(TestZpf, ParamWithoutNominalDefaultsPersistsAcrossTransitions)
{
  // state_1 sets inflation; no nominal_defaults entry for inflation. The
  // cross-state reset cannot restore (gap warned at config-load); inflation
  // legitimately retains state_1's value across 1→2.
  ASSERT_TRUE(createFilter(
      {1, 2},
  {
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.speed", 0.3),
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_1.zpf_target_node.inflation", 0.8),
    rclcpp::Parameter(
          std::string(kFilterName) + ".state_2.zpf_target_node.speed", 0.5),
    rclcpp::Parameter(
          std::string(kFilterName) + ".nominal_defaults.zpf_target_node.speed", 1.0),
    // No nominal_defaults entry for inflation — gap warned at config-load.
      },
      1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 0.3;}));
  ASSERT_TRUE(waitFor([this]() {return target_node_->getInflation() == 0.8;}));

  rePublishMask(2);
  runProcess();
  ASSERT_TRUE(waitFor([this]() {return target_node_->getSpeed() == 0.5;}));
  EXPECT_DOUBLE_EQ(target_node_->getInflation(), 0.8)
    << "param without nominal_defaults legitimately persists across N→M";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
