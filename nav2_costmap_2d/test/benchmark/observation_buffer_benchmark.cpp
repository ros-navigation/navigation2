// Copyright (c) 2025 Fabian Koenig
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

#include <random>
#include "benchmark/benchmark.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
static constexpr const char *global_frame{"map"};
static constexpr const char *sensor_frame{"sensor"};

class TestObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void addObservationBuffer(std::shared_ptr<nav2_costmap_2d::ObservationBuffer> buffer)
  {
    observation_buffers_.push_back(buffer);
    marking_buffers_.push_back(buffer);
  }

  static void addObstacleLayer(
    nav2_costmap_2d::LayeredCostmap & layers,
    tf2_ros::Buffer & tf, nav2::LifecycleNode::SharedPtr node,
    std::shared_ptr<TestObstacleLayer> & olayer,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
  {
    olayer = std::make_shared<TestObstacleLayer>();
    olayer->initialize(&layers, "obstacles", &tf, node, callback_group);
    layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(olayer));
  }
};
}  // namespace

class ObservationBufferFixture : public benchmark::Fixture
{
public:
  void SetUp(benchmark::State & /*state*/)
  {
    node_ = std::make_shared<nav2::LifecycleNode>(
        "observation_buffer_test_node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    buffer_ =
      std::make_shared<nav2_costmap_2d::ObservationBuffer>(node_, "/observations", 10.0,
      0.1, 0.0, std::numeric_limits<double>::max(), 10.0, 1.0, 5.0, 2.0, *tf_buffer_, global_frame,
      sensor_frame, tf2::Duration{0});

    auto clock_handle = node_->get_clock()->get_clock_handle();
    auto ret = rcl_enable_ros_time_override(clock_handle);
    ret = rcl_set_ros_time_override(clock_handle, 0);
    (void)ret;
    geometry_msgs::msg::TransformStamped tf_msg{};
    tf_msg.child_frame_id = sensor_frame;
    tf_msg.header.frame_id = global_frame;
    tf_msg.transform = tf2::toMsg(tf2::Transform::getIdentity());
    tf_buffer_->setTransform(tf_msg, "", true);

    layers_.resizeMap(100, 100, 0.1, 0, 0);
    TestObstacleLayer::addObstacleLayer(layers_, *tf_buffer_, node_, olayer_);
    olayer_->addObservationBuffer(buffer_);

    constexpr auto num_points{1024 * 64};
    test_clouds_[0] = generateRandomPointCloud(num_points, rclcpp::Time{1, 0});
    test_clouds_[1] = generateRandomPointCloud(num_points, rclcpp::Time{5, 0});
    test_clouds_[2] = generateRandomPointCloud(num_points, rclcpp::Time{10, 0});
  }

  void TearDown(benchmark::State & /*state*/)
  {
    node_->shutdown();
    buffer_.reset();
    tf_buffer_.reset();
    node_.reset();
    olayer_.reset();
  }

  void
  setClock(const rclcpp::Time & time)
  {
    auto clock_handle = node_->get_clock()->get_clock_handle();
    auto ret = rcl_set_ros_time_override(clock_handle, time.nanoseconds());
    (void)ret;
  }

  static sensor_msgs::msg::PointCloud2 generateRandomPointCloud(
    size_t num_points,
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(num_points);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    std::random_device rand_dev{};
    std::mt19937 generator{rand_dev()};
    std::uniform_int_distribution<int> distr{0, 10000};
    for (auto i = 0u; i < num_points; ++i) {
      *iter_x = distr(generator) / 100.0;
      *iter_y = distr(generator) / 100.0;
      *iter_z = distr(generator) / 100.0;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    cloud.header.frame_id = sensor_frame;
    cloud.header.stamp = stamp;

    return cloud;
  }

  nav2::LifecycleNode::SharedPtr node_{};
  tf2_ros::Buffer::SharedPtr tf_buffer_;
  nav2_costmap_2d::LayeredCostmap layers_{global_frame, false, false};
  std::shared_ptr<nav2_costmap_2d::ObservationBuffer> buffer_{};
  std::shared_ptr<TestObstacleLayer> olayer_;
  std::array<sensor_msgs::msg::PointCloud2, 3> test_clouds_;
};

BENCHMARK_F(ObservationBufferFixture, AddCloudsAndGetObersvations)(benchmark::State & state)
{
  for (auto _ : state) {
    setClock(rclcpp::Time{1, 0});
    buffer_->bufferCloud(test_clouds_[0]);
    setClock(rclcpp::Time{5, 0});
    buffer_->bufferCloud(test_clouds_[1]);
    setClock(rclcpp::Time{10, 0});
    buffer_->bufferCloud(test_clouds_[2]);
    std::vector<nav2_costmap_2d::Observation::ConstSharedPtr> observations{};
    // The observation_keep_time is 10s and the age of the oldest cloud is 9s
    // so all 3 added observations should be returned
    layers_.updateMap(0, 0, 0);
  }
}

int main(int argc, char **argv)
{
  // Initialize the system
  std::cerr << "before init" << std::endl;
  rclcpp::init(argc, argv);
  std::cerr << "after init" << std::endl;
  benchmark::Initialize(&argc, argv);

  // Run the benchmarks
  benchmark::RunSpecifiedBenchmarks();

  // Shutdown
  rclcpp::shutdown();

  return 0;
}
