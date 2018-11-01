/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <gtest/gtest.h>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/time_sequencer.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/chain.h"
#include "sensor_msgs/msg/imu.hpp"

using namespace message_filters;
typedef sensor_msgs::msg::Imu Msg;
typedef std::shared_ptr<sensor_msgs::msg::Imu const> MsgConstPtr;
typedef std::shared_ptr<sensor_msgs::msg::Imu> MsgPtr;

class Helper
{
public:
  Helper()
  : count_(0)
  {}

  void cb(const MsgConstPtr&)
  {
    ++count_;
  }

  void cb2(const MsgConstPtr&, const MsgConstPtr&)
  {
    ++count_;
  }
  int32_t count_;
};

static void fuzz_msg(MsgPtr msg) {
    static std::random_device seeder;
    std::mt19937 gen(seeder());
    std::uniform_real_distribution<float> distr(1.0, 3.0);
    msg->linear_acceleration.x = distr(gen);
    msg->linear_acceleration.y = distr(gen);
    msg->linear_acceleration.z = distr(gen);
}

TEST(TimeSequencer, fuzz_sequencer)
{
  rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("test_node");
  TimeSequencer<Msg> seq(rclcpp::Duration(0, 10000000), rclcpp::Duration(0, 1000000), 10, nh);
  Helper h;
  seq.registerCallback(std::bind(&Helper::cb, &h, _1));
  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  auto msg = std::make_shared<Msg>();
  while ((ros_clock.now() - start) < rclcpp::Duration(5.0, 0)) {
    h.count_ = 0;
    fuzz_msg(msg);
    msg->header.stamp = ros_clock.now();
    seq.add(msg);

    rclcpp::Rate(20).sleep();
    ASSERT_EQ(h.count_, 0);
    rclcpp::spin_some(seq.get_node());
    rclcpp::Rate(100).sleep();
    rclcpp::spin_some(seq.get_node());
    ASSERT_EQ(h.count_, 1);
  }
}

TEST(TimeSynchronizer, fuzz_synchronizer)
{
  TimeSynchronizer<Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb2, &h, _1, _2));

  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  auto msg1 = std::make_shared<Msg>();
  auto msg2 = std::make_shared<Msg>();
  while ((ros_clock.now() - start) < rclcpp::Duration(5.0, 0))
  {
    h.count_ = 0;
    fuzz_msg(msg1);
    msg1->header.stamp = rclcpp::Clock().now();
    fuzz_msg(msg2);
    msg2->header.stamp = msg1->header.stamp;
    sync.add0(msg1);
    ASSERT_EQ(h.count_, 0);
    sync.add1(msg2);
    ASSERT_EQ(h.count_, 1);
    rclcpp::Rate(50).sleep();
  }
}

TEST(Subscriber, fuzz_subscriber)
{
  auto nh = std::make_shared<rclcpp::Node>("test_node");
  Helper h;
  Subscriber<Msg> sub(nh.get(), "test_topic");
  sub.registerCallback(std::bind(&Helper::cb, &h, _1));
  auto pub = nh->create_publisher<Msg>("test_topic");
  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  auto msg = std::make_shared<Msg>();
  while ((ros_clock.now() - start) < rclcpp::Duration(5.0, 0))
  {
    h.count_ = 0;
    fuzz_msg(msg);
    msg->header.stamp = ros_clock.now();
    pub->publish(msg);
    rclcpp::Rate(50).sleep();
    rclcpp::spin_some(nh);
    ASSERT_EQ(h.count_, 1);
  }
  rclcpp::spin_some(nh);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}


