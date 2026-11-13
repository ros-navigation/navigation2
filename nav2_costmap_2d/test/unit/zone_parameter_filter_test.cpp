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

#include "rclcpp/duration.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/srv/get_parameter_types.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
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

// Full parameter name under the filter's namespace.
std::string fp(const std::string & suffix)
{
  return std::string(kFilterName) + "." + suffix;
}

// Appends the declared-config overrides for one explicit
// {node, parameter, value} entry rooted at fp(prefix). Used both for
// `<state>.<override_name>` entries and `nominal_defaults.<name>` entries.
void addEntry(
  std::vector<rclcpp::Parameter> & cfg, const std::string & prefix,
  const std::string & target_node, const std::string & parameter,
  const rclcpp::ParameterValue & value)
{
  cfg.emplace_back(fp(prefix + ".node"), target_node);
  cfg.emplace_back(fp(prefix + ".parameter"), parameter);
  cfg.push_back(rclcpp::Parameter(fp(prefix + ".value"), value));
}
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

// A second explicit target: the declared config routes overrides by explicit
// `node` + `parameter` fields, so two overrides in one state can address two
// different nodes without any name disambiguation.
class SecondTargetNode : public rclcpp::Node
{
public:
  SecondTargetNode()
  : rclcpp::Node("zpf_second_target")
  {
    declare_parameter("speed", 1.0);
  }

  double getSpeed() {return get_parameter("speed").as_double();}
};

// Serves the parameter services of a node but never answers them, as a node
// whose executor is stuck would.
class SilentTargetNode : public rclcpp::Node
{
public:
  SilentTargetNode()
  : rclcpp::Node("zpf_silent_target", rclcpp::NodeOptions().start_parameter_services(false))
  {
    ignore<rcl_interfaces::srv::GetParameters>("get_parameters");
    ignore<rcl_interfaces::srv::GetParameterTypes>("get_parameter_types");
    ignore<rcl_interfaces::srv::SetParameters>("set_parameters", &set_requests);
    ignore<rcl_interfaces::srv::SetParametersAtomically>("set_parameters_atomically");
    ignore<rcl_interfaces::srv::ListParameters>("list_parameters");
    ignore<rcl_interfaces::srv::DescribeParameters>("describe_parameters");
  }

  int set_requests{0};

private:
  template<typename ServiceT>
  void ignore(const std::string & service, int * requests = nullptr)
  {
    // A callback taking the request header defers the response, which is
    // never sent
    services_.push_back(
      create_service<ServiceT>(
        std::string(get_name()) + "/" + service,
        [requests](
          const std::shared_ptr<rmw_request_id_t>,
          const std::shared_ptr<typename ServiceT::Request>) {
          if (requests) {
            ++*requests;
          }
        }));
  }

  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
};

// Refuses every parameter change while `refuse` is set, and counts the
// changes it is asked for
class RefusingTargetNode : public rclcpp::Node
{
public:
  RefusingTargetNode()
  : rclcpp::Node("zpf_refusing_target")
  {
    declare_parameter("speed", 1.0);
    on_set_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {
        ++attempts;
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = !refuse;
        result.reason = refuse ? "refused by the test" : "";
        return result;
      });
  }

  double getSpeed() {return get_parameter("speed").as_double();}

  bool refuse{true};
  int attempts{0};

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_handle_;
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
    if (second_target_node_) {
      target_executor_.remove_node(second_target_node_);
      second_target_node_.reset();
    }
    if (silent_target_node_) {
      target_executor_.remove_node(silent_target_node_);
      silent_target_node_.reset();
    }
    if (refusing_target_node_) {
      target_executor_.remove_node(refusing_target_node_);
      refusing_target_node_.reset();
    }
    target_executor_.remove_node(target_node_);
    target_node_.reset();
  }

  // Builds a ZPF host node + filter + info/mask publishers. The filter's
  // state machine is injected as DECLARED configuration through
  // NodeOptions::parameter_overrides:
  //   <filter>.states                       : string array of state names
  //   <filter>.<state>.id                   : int64 mask value for the state
  //   <filter>.<state>.overrides            : string array of override names
  //   <filter>.<state>.<override>.node      : explicit target node
  //   <filter>.<state>.<override>.parameter : parameter on that node
  //   <filter>.<state>.<override>.value     : dynamically typed value
  //   <filter>.nominal_defaults             : string array; entries as above
  // Spins until the filter becomes active or 2s pass. Optional info_type
  // allows the wrong-type test to inject a non-ZPF info publisher.
  bool createFilter(
    const std::vector<rclcpp::Parameter> & filter_config,
    int8_t mask_fill_value,
    const std::string & state_event_topic = "",
    uint8_t info_type = nav2_costmap_2d::ZONE_PARAMETER_FILTER)
  {
    rclcpp::NodeOptions opts;
    std::vector<rclcpp::Parameter> all_overrides = {
      rclcpp::Parameter(fp("filter_info_topic"), std::string(kInfoTopic)),
      rclcpp::Parameter(fp("transform_tolerance"), 0.5),
    };
    if (!state_event_topic.empty()) {
      all_overrides.emplace_back(fp("state_event_topic"), state_event_topic);
    }
    for (const auto & p : filter_config) {
      all_overrides.push_back(p);
    }
    opts.parameter_overrides(all_overrides);

    node_ = std::make_shared<nav2::LifecycleNode>("zpf_test_host", opts);
    node_executor_.add_node(node_->get_node_base_interface());

    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    tf_buffer_ = nav2::create_transform_buffer(node_);
    tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

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
  bool waitForCond(Pred pred, std::chrono::milliseconds timeout = 1500ms)
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
  std::shared_ptr<SecondTargetNode> second_target_node_;
  std::shared_ptr<SilentTargetNode> silent_target_node_;
  std::shared_ptr<RefusingTargetNode> refusing_target_node_;
  std::shared_ptr<StateEventSubscriber> state_event_sub_;
  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  nav2::TransformBuffer::SharedPtr tf_buffer_;
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
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.5));

  EXPECT_FALSE(createFilter(cfg, 1, "", nav2_costmap_2d::BINARY_FILTER));
}

TEST_F(TestZpf, State1AppliesParameterAndResetFilterDeactivates)
{
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.5));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active within 2s";

  EXPECT_DOUBLE_EQ(target_node_->getSpeed(), 1.0);

  runProcess();
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getSpeed() == 0.5;}))
    << "Target node 'speed' did not become 0.5 within 1.5s";

  EXPECT_DOUBLE_EQ(target_node_->getInflation(), 0.5);

  filter_->resetFilter();
  EXPECT_FALSE(filter_->isActive())
    << "resetFilter() must clear filter_info_received_ and filter_mask_";
}

TEST_F(TestZpf, State0ResetsToNominalDefaults)
{
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
    rclcpp::Parameter(fp("nominal_defaults"), std::vector<std::string>{"speed_nominal"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.3));
  addEntry(cfg, "nominal_defaults.speed_nominal", "zpf_target_node", "speed",
    rclcpp::ParameterValue(1.0));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getSpeed() == 0.3;}));

  rePublishMask(0);
  runProcess();
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getSpeed() == 1.0;}))
    << "speed never restored to nominal default 1.0 after state 0";
}

TEST_F(TestZpf, UnknownStateKeepsTheParameters)
{
  // Mask carries state 2, but only state id 1 is configured — applyState
  // logs it, nothing throws, no parameter changes and the state is not
  // entered.
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.3));

  ASSERT_TRUE(createFilter(cfg, 2, "/zpf_state")) << "Filter did not become active";

  EXPECT_NO_THROW(runProcess());
  spinFor(250ms);
  EXPECT_DOUBLE_EQ(target_node_->getSpeed(), 1.0)
    << "speed must NOT change on an unknown state";
  EXPECT_FALSE(state_event_sub_->received()) << "an unknown state must not be entered";
}

TEST_F(TestZpf, StateEventPublishedOnTransition)
{
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.3));

  ASSERT_TRUE(createFilter(cfg, 1, "/zpf_state")) << "Filter did not become active";

  ASSERT_FALSE(state_event_sub_->received());

  // The state is entered, and its event published, once its update succeeds
  ASSERT_TRUE(
    waitForCond(
      [this]() {
        runProcess();
        return state_event_sub_->received();
      }));
  EXPECT_EQ(state_event_sub_->lastState(), 1u);
}

TEST_F(TestZpf, ProcessHotPathReturnsPromptlyEvenWithUnreachableTargets)
{
  // The override explicitly names a node that does not exist. The async
  // parameter client is built at config-load; the update waits for its
  // parameter service. process() must neither block nor throw.
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"remote_zone"}),
    rclcpp::Parameter(fp("remote_zone.id"), 1),
    rclcpp::Parameter(fp("remote_zone.setpoints"), std::vector<std::string>{"remote_speed"}),
  };
  addEntry(cfg, "remote_zone.remote_speed", "nonexistent_node", "foo",
    rclcpp::ParameterValue(0.5));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

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
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.5));

  ASSERT_TRUE(createFilter(cfg, static_cast<int8_t>(-1))) << "Filter did not become active";

  runProcess();
  spinFor(250ms);
  EXPECT_DOUBLE_EQ(target_node_->getSpeed(), 1.0);
}

TEST_F(TestZpf, RobotOutsideMaskResetsToState0)
{
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
    rclcpp::Parameter(fp("nominal_defaults"), std::vector<std::string>{"speed_nominal"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.4));
  addEntry(cfg, "nominal_defaults.speed_nominal", "zpf_target_node", "speed",
    rclcpp::ParameterValue(1.0));

  ASSERT_TRUE(createFilter(cfg, 1, "/zpf_state")) << "Filter did not become active";

  ASSERT_TRUE(
    waitForCond(
      [this]() {
        runProcess();
        return target_node_->getSpeed() == 0.4 && state_event_sub_->received();
      }));
  EXPECT_EQ(state_event_sub_->lastState(), 1u);

  ASSERT_TRUE(
    waitForCond(
      [this]() {
        runProcess(100.0, 100.0);
        return target_node_->getSpeed() == 1.0;
      })) << "speed never restored to nominal after out-of-mask pose";

  // Leaving the mask restores the parameters without a state event
  for (int i = 0; i < 10; ++i) {
    runProcess(100.0, 100.0);
    spinFor(20ms);
  }
  EXPECT_EQ(state_event_sub_->lastState(), 1u) << "leaving the mask must not publish a state event";
}

TEST_F(TestZpf, ExplicitNodeRoutingAppliesOverridesToBothTargets)
{
  // The declared config carries an explicit `node` + `parameter` per
  // override, so nothing is inferred from key names (the old longest-prefix
  // target_nodes matching is gone by design). Two overrides in one state
  // addressing two different nodes must BOTH be routed and applied.
  second_target_node_ = std::make_shared<SecondTargetNode>();
  target_executor_.add_node(second_target_node_);

  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(
      fp("slow_zone.setpoints"),
      std::vector<std::string>{"host_speed", "second_speed"}),
  };
  addEntry(cfg, "slow_zone.host_speed", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.7));
  addEntry(cfg, "slow_zone.second_speed", "zpf_second_target", "speed",
    rclcpp::ParameterValue(0.25));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getSpeed() == 0.7;}))
    << "first explicit target never received speed 0.7";
  ASSERT_TRUE(waitForCond([this]() {return second_target_node_->getSpeed() == 0.25;}))
    << "second explicit target never received speed 0.25";
}

TEST_F(TestZpf, InfoAndMaskRePublishUpdateSubscriptions)
{
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone", "crawl_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
    rclcpp::Parameter(fp("crawl_zone.id"), 2),
    rclcpp::Parameter(fp("crawl_zone.setpoints"), std::vector<std::string>{"speed_override"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.5));
  addEntry(cfg, "crawl_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.2));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getSpeed() == 0.5;}));

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
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getSpeed() == 0.2;}))
    << "state-2 speed never landed after mask re-publish";
}

TEST_F(TestZpf, EmptyAndInvalidConfigEdgesDoNotCrash)
{
  // Declared-config edge cases must be rejected entry-by-entry at
  // config-load without breaking the valid remainder:
  //  - state id 0 (reserved for reset) and id 256 (> 255) — state skipped;
  //  - an override missing its `node` field — entry skipped;
  //  - an override whose `value` is never provided — entry skipped;
  //  - entry params present but NOT listed in `overrides` — never read.
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(
      fp("states"),
      std::vector<std::string>{"reserved_zone", "big_zone", "slow_zone"}),
    rclcpp::Parameter(fp("reserved_zone.id"), 0),
    rclcpp::Parameter(fp("big_zone.id"), 256),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(
      fp("slow_zone.setpoints"),
      std::vector<std::string>{"bad_entry", "no_value", "good"}),
    // bad_entry: `node` never provided (defaults empty) -> skipped.
    rclcpp::Parameter(fp("slow_zone.bad_entry.parameter"), std::string("speed")),
    rclcpp::Parameter(fp("slow_zone.bad_entry.value"), 9.9),
    // no_value: named target but `.value` missing -> skipped.
    rclcpp::Parameter(fp("slow_zone.no_value.node"), std::string("zpf_target_node")),
    rclcpp::Parameter(fp("slow_zone.no_value.parameter"), std::string("inflation")),
    // ghost: fully formed but absent from `overrides` -> never read.
    rclcpp::Parameter(fp("slow_zone.ghost.node"), std::string("zpf_target_node")),
    rclcpp::Parameter(fp("slow_zone.ghost.parameter"), std::string("inflation")),
    rclcpp::Parameter(fp("slow_zone.ghost.value"), 9.9),
  };
  addEntry(cfg, "slow_zone.good", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.5));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getSpeed() == 0.5;}))
    << "valid override did not apply alongside invalid states + malformed entries";
  EXPECT_DOUBLE_EQ(target_node_->getInflation(), 0.5)
    << "skipped/unlisted entries must not touch 'inflation'";
}

TEST_F(TestZpf, RemovedFailurePolicyParameterIsIgnored)
{
  // The rework removed the on_param_set_failure policy. A leftover
  // `on_param_set_failure: warn` in YAML is simply an undeclared, unused
  // override — config-load must not raise, and a failed set is handled as
  // any other: reported and retried, never thrown.
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("on_param_set_failure"), std::string("warn")),
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"danger_zone"}),
    rclcpp::Parameter(fp("danger_zone.id"), 1),
    rclcpp::Parameter(fp("danger_zone.setpoints"), std::vector<std::string>{"ro_speed"}),
  };
  addEntry(cfg, "danger_zone.ro_speed", "zpf_target_node", "readonly_speed",
    rclcpp::ParameterValue(0.5));

  ASSERT_TRUE(createFilter(cfg, 1))
    << "Filter did not become active (leftover policy param must not break config-load)";

  auto drive_and_drain = [this]() {
      runProcess();
      spinFor(300ms);  // drain: failed result becomes ready
      runProcess();    // checkPendingParameterUpdates surfaces it here
    };
  EXPECT_NO_THROW(drive_and_drain());
  EXPECT_DOUBLE_EQ(target_node_->get_parameter("readonly_speed").as_double(), 1.0);
}

TEST_F(TestZpf, StateIsEnteredOnlyOnceItsUpdatesSucceed)
{
  // The target refuses the set: process() reports it without throwing, does
  // not enter the state, and applies it again every parameter_update_timeout
  // until the target accepts it. Only then is the state entered.
  refusing_target_node_ = std::make_shared<RefusingTargetNode>();
  target_executor_.add_node(refusing_target_node_);

  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("parameter_update_timeout"), 0.2),
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_refusing_target", "speed",
    rclcpp::ParameterValue(0.5));

  ASSERT_TRUE(createFilter(cfg, 1, "/zpf_state")) << "Filter did not become active";

  ASSERT_TRUE(
    waitForCond(
      [this]() {
        EXPECT_NO_THROW(runProcess());
        return refusing_target_node_->attempts >= 2;
      })) << "the refused state was not applied again";
  EXPECT_DOUBLE_EQ(refusing_target_node_->getSpeed(), 1.0);
  EXPECT_FALSE(state_event_sub_->received()) << "a state whose update failed must not be entered";

  refusing_target_node_->refuse = false;
  ASSERT_TRUE(
    waitForCond(
      [this]() {
        EXPECT_NO_THROW(runProcess());
        return state_event_sub_->received();
      })) << "the state was not entered once the target accepted its update";
  EXPECT_EQ(state_event_sub_->lastState(), 1u);
  EXPECT_DOUBLE_EQ(refusing_target_node_->getSpeed(), 0.5);
}

TEST_F(TestZpf, PartlyAppliedStateIsRestoredWhenLeft)
{
  // State 1 lands on zpf_target_node but is refused by zpf_refusing_target,
  // so it is not entered. Leaving it for state 2, which only sets the
  // refusing target, still restores what state 1 set on zpf_target_node.
  refusing_target_node_ = std::make_shared<RefusingTargetNode>();
  target_executor_.add_node(refusing_target_node_);

  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("parameter_update_timeout"), 0.2),
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone", "crawl_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(
      fp("slow_zone.setpoints"),
      std::vector<std::string>{"host_speed", "refusing_speed"}),
    rclcpp::Parameter(fp("crawl_zone.id"), 2),
    rclcpp::Parameter(fp("crawl_zone.setpoints"), std::vector<std::string>{"refusing_speed"}),
    rclcpp::Parameter(
      fp("nominal_defaults"),
      std::vector<std::string>{"host_speed", "refusing_speed"}),
  };
  addEntry(cfg, "slow_zone.host_speed", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.3));
  addEntry(cfg, "slow_zone.refusing_speed", "zpf_refusing_target", "speed",
    rclcpp::ParameterValue(0.5));
  addEntry(cfg, "crawl_zone.refusing_speed", "zpf_refusing_target", "speed",
    rclcpp::ParameterValue(0.2));
  addEntry(cfg, "nominal_defaults.host_speed", "zpf_target_node", "speed",
    rclcpp::ParameterValue(1.0));
  addEntry(cfg, "nominal_defaults.refusing_speed", "zpf_refusing_target", "speed",
    rclcpp::ParameterValue(1.0));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  ASSERT_TRUE(
    waitForCond(
      [this]() {
        EXPECT_NO_THROW(runProcess());
        return target_node_->getSpeed() == 0.3 && refusing_target_node_->attempts >= 1;
      })) << "state 1 was not partly applied";

  refusing_target_node_->refuse = false;
  rePublishMask(2);
  ASSERT_TRUE(
    waitForCond(
      [this]() {
        EXPECT_NO_THROW(runProcess());
        return refusing_target_node_->getSpeed() == 0.2 && target_node_->getSpeed() == 1.0;
      })) << "leaving state 1 did not restore what it set on zpf_target_node";
}

TEST_F(TestZpf, TargetStartingAfterTheTimeoutReceivesItsUpdate)
{
  // The target node appears after parameter_update_timeout: the expired
  // update is reported without throwing, and the retry reaches the node.
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("parameter_update_timeout"), 0.2),
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_second_target", "speed",
    rclcpp::ParameterValue(0.25));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  EXPECT_NO_THROW(runProcess());
  spinFor(300ms);
  EXPECT_NO_THROW(runProcess());

  second_target_node_ = std::make_shared<SecondTargetNode>();
  target_executor_.add_node(second_target_node_);
  ASSERT_TRUE(
    waitForCond(
      [this]() {
        EXPECT_NO_THROW(runProcess());
        return second_target_node_->getSpeed() == 0.25;
      })) << "the update that expired never reached the target once it started";
}

TEST_F(TestZpf, UnansweredSetIsSentAgain)
{
  // The target node is up but never answers: once parameter_update_timeout
  // passes, the set is reported without throwing and sent again.
  silent_target_node_ = std::make_shared<SilentTargetNode>();
  target_executor_.add_node(silent_target_node_);

  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("parameter_update_timeout"), 0.2),
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_silent_target", "speed",
    rclcpp::ParameterValue(0.5));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  ASSERT_TRUE(
    waitForCond(
      [this]() {
        EXPECT_NO_THROW(runProcess());
        return silent_target_node_->set_requests >= 2;
      })) << "the unanswered set was not sent again";
}

TEST_F(TestZpf, RetryDoesNotUndoANewerState)
{
  // State 1's update expired, and state 1 waits to be applied again, when the
  // robot enters state 2: the target ends on state 2's value.
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("parameter_update_timeout"), 0.5),
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone", "crawl_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
    rclcpp::Parameter(fp("crawl_zone.id"), 2),
    rclcpp::Parameter(fp("crawl_zone.setpoints"), std::vector<std::string>{"speed_override"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_second_target", "speed",
    rclcpp::ParameterValue(0.25));
  addEntry(cfg, "crawl_zone.speed_override", "zpf_second_target", "speed",
    rclcpp::ParameterValue(0.1));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  runProcess();
  spinFor(600ms);
  runProcess();  // state 1's update expires
  rePublishMask(2);
  runProcess();  // state 2's update is issued before state 1 is due again

  second_target_node_ = std::make_shared<SecondTargetNode>();
  target_executor_.add_node(second_target_node_);
  ASSERT_TRUE(
    waitForCond(
      [this]() {
        runProcess();
        return second_target_node_->getSpeed() == 0.1;
      })) << "state 2's update never reached the target";

  // Past the time state 1 would have been applied again
  for (int i = 0; i < 60; ++i) {
    runProcess();
    spinFor(20ms);
  }
  EXPECT_DOUBLE_EQ(second_target_node_->getSpeed(), 0.1);
}

TEST_F(TestZpf, TargetStartingLateReceivesItsUpdate)
{
  // The target node appears after the update is issued, within
  // parameter_update_timeout: the update reaches it, and nothing fails.
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("parameter_update_timeout"), 2.0),
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"speed_override"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_second_target", "speed",
    rclcpp::ParameterValue(0.25));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  EXPECT_NO_THROW(runProcess());
  second_target_node_ = std::make_shared<SecondTargetNode>();
  target_executor_.add_node(second_target_node_);
  ASSERT_TRUE(
    waitForCond(
      [this]() {
        runProcess();
        return second_target_node_->getSpeed() == 0.25;
      })) << "update issued before the target started never reached it";
}

TEST_F(TestZpf, CrossStateTransitionResetsParamTouchedByNOnly)
{
  // state 1 sets both speed AND inflation; state 2 sets speed only.
  // Transition 1→2 must restore inflation to its nominal_defaults value.
  // The in-both param (speed) must land at state-2's value, not the nominal.
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone", "fast_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(
      fp("slow_zone.setpoints"),
      std::vector<std::string>{"speed_override", "inflation_override"}),
    rclcpp::Parameter(fp("fast_zone.id"), 2),
    rclcpp::Parameter(fp("fast_zone.setpoints"), std::vector<std::string>{"speed_override"}),
    rclcpp::Parameter(
      fp("nominal_defaults"),
      std::vector<std::string>{"speed_nominal", "inflation_nominal"}),
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.3));
  addEntry(cfg, "slow_zone.inflation_override", "zpf_target_node", "inflation",
    rclcpp::ParameterValue(0.8));
  addEntry(cfg, "fast_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.5));
  addEntry(cfg, "nominal_defaults.speed_nominal", "zpf_target_node", "speed",
    rclcpp::ParameterValue(1.0));
  addEntry(cfg, "nominal_defaults.inflation_nominal", "zpf_target_node", "inflation",
    rclcpp::ParameterValue(2.0));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getSpeed() == 0.3;}));
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getInflation() == 0.8;}));

  rePublishMask(2);
  runProcess();
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getSpeed() == 0.5;}))
    << "in-both param must land at state-2's value, not state-1's nominal";
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getInflation() == 2.0;}))
    << "inflation must be reset to nominal_defaults on 1→2 transition";
}

TEST_F(TestZpf, ParamWithoutNominalDefaultsPersistsAcrossTransitions)
{
  // state 1 sets inflation; no nominal_defaults entry for inflation. The
  // cross-state reset cannot restore it (gap warned at config-load);
  // inflation legitimately retains state-1's value across 1→2.
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone", "fast_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(
      fp("slow_zone.setpoints"),
      std::vector<std::string>{"speed_override", "inflation_override"}),
    rclcpp::Parameter(fp("fast_zone.id"), 2),
    rclcpp::Parameter(fp("fast_zone.setpoints"), std::vector<std::string>{"speed_override"}),
    rclcpp::Parameter(fp("nominal_defaults"), std::vector<std::string>{"speed_nominal"}),
    // No nominal_defaults entry for inflation — gap warned at config-load.
  };
  addEntry(cfg, "slow_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.3));
  addEntry(cfg, "slow_zone.inflation_override", "zpf_target_node", "inflation",
    rclcpp::ParameterValue(0.8));
  addEntry(cfg, "fast_zone.speed_override", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.5));
  addEntry(cfg, "nominal_defaults.speed_nominal", "zpf_target_node", "speed",
    rclcpp::ParameterValue(1.0));

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getSpeed() == 0.3;}));
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getInflation() == 0.8;}));

  rePublishMask(2);
  runProcess();
  ASSERT_TRUE(waitForCond([this]() {return target_node_->getSpeed() == 0.5;}));
  EXPECT_DOUBLE_EQ(target_node_->getInflation(), 0.8)
    << "param without nominal_defaults legitimately persists across N→M";
}


class ZoneParameterFilterTFTestable : public nav2_costmap_2d::ZoneParameterFilter
{
public:
  using ZoneParameterFilter::maskCallback;
  int state() const {return current_state_;}
};

TEST(ZoneParameterFilterTF, StaleTransformPreservesState)
{
  auto node = std::make_shared<nav2::LifecycleNode>("tf_test");
  node->declare_parameter("filter.filter_info_topic", "filter_info");
  node->declare_parameter("filter.transform_staleness_threshold", 0.5);
  node->declare_parameter("filter.states", std::vector<std::string>{"restricted"});
  node->declare_parameter("filter.restricted.id", 1);
  auto buffer = nav2::create_transform_buffer(node);
  nav2_costmap_2d::LayeredCostmap layers("odom", false, false);
  ZoneParameterFilterTFTestable filter;
  filter.initialize(&layers, "filter", buffer.get(), node, nullptr);
  filter.initializeFilter("filter_info");
  auto mask = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  mask->header.frame_id = "map";
  mask->info.width = 2;
  mask->info.height = 1;
  mask->info.resolution = 1.0;
  mask->info.origin.orientation.w = 1.0;
  mask->data = {1, 0};
  filter.maskCallback(mask);
  geometry_msgs::msg::Pose pose;
  pose.position.x = pose.position.y = 0.5;
  pose.orientation.w = 1.0;
  nav2_costmap_2d::Costmap2D grid(2, 1, 1.0, 0.0, 0.0, 0);
  filter.process(grid, 0, 0, 2, 1, pose);
  EXPECT_EQ(filter.state(), 0);  // Missing TF cannot trigger a transition.

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.rotation.w = 1.0;
  transform.header.stamp = node->now() - rclcpp::Duration::from_seconds(5.0);
  ASSERT_TRUE(buffer->setTransform(transform, "test", false));
  filter.process(grid, 0, 0, 2, 1, pose);
  EXPECT_EQ(filter.state(), 0);  // Stale TF cannot enter the marked zone.

  transform.header.stamp = node->now();
  ASSERT_TRUE(buffer->setTransform(transform, "test", false));
  filter.process(grid, 0, 0, 2, 1, pose);
  EXPECT_EQ(filter.state(), 1);

  buffer->clear();
  transform.header.stamp = node->now() - rclcpp::Duration::from_seconds(5.0);
  transform.transform.translation.x = 1.0;
  ASSERT_TRUE(buffer->setTransform(transform, "test", false));
  filter.process(grid, 0, 0, 2, 1, pose);
  EXPECT_EQ(filter.state(), 1);  // Stale TF must not reset the active state.

  transform.header.stamp = node->now();
  ASSERT_TRUE(buffer->setTransform(transform, "test", false));
  filter.process(grid, 0, 0, 2, 1, pose);
  EXPECT_EQ(filter.state(), 0);  // Fresh TF permits the transition.

  buffer->clear();
  transform.header.stamp = node->now() - rclcpp::Duration::from_seconds(5.0);
  transform.transform.translation.x = 0.0;
  ASSERT_TRUE(buffer->setTransform(transform, "test", true));
  filter.process(grid, 0, 0, 2, 1, pose);
  EXPECT_EQ(filter.state(), 1);  // Static transforms do not expire.

  mask = std::make_shared<nav_msgs::msg::OccupancyGrid>(*mask);
  mask->header.frame_id = "odom";
  mask->data = {0, 0};
  filter.maskCallback(mask);
  filter.process(grid, 0, 0, 2, 1, pose);
  EXPECT_EQ(filter.state(), 0);  // Same-frame poses need no TF.
  filter.resetFilter();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
