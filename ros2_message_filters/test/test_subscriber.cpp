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

#include <rclcpp/rclcpp.hpp>
#include "message_filters/subscriber.h"
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

  int32_t count_;
};

TEST(Subscriber, simple)
{
  auto nh = std::make_shared<rclcpp::Node>("test_node");
  Helper h;
  Subscriber<Msg> sub(nh.get(), "test_topic");
  sub.registerCallback(std::bind(&Helper::cb, &h, _1));
  auto pub = nh->create_publisher<Msg>("test_topic");
  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  while (h.count_ == 0 && (ros_clock.now() - start) < rclcpp::Duration(1.0, 0))
  {
    pub->publish(std::make_shared<Msg>());
    rclcpp::Rate(50).sleep();
    rclcpp::spin_some(nh);
  }

  ASSERT_GT(h.count_, 0);
}

TEST(Subscriber, subUnsubSub)
{
  auto nh = std::make_shared<rclcpp::Node>("test_node");
  Helper h;
  Subscriber<Msg> sub(nh.get(), "test_topic");
  sub.registerCallback(std::bind(&Helper::cb, &h, _1));
  auto pub = nh->create_publisher<Msg>("test_topic");

  sub.unsubscribe();
  sub.subscribe();

  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  while (h.count_ == 0 && (ros_clock.now() - start) < rclcpp::Duration(1.0, 0))
  {
    pub->publish(std::make_shared<Msg>());
    rclcpp::Rate(50).sleep();
    rclcpp::spin_some(nh);
  }

  ASSERT_GT(h.count_, 0);
}

TEST(Subscriber, subInChain)
{
  auto nh = std::make_shared<rclcpp::Node>("test_node");
  Helper h;
  Chain<Msg> c;
  c.addFilter(std::make_shared<Subscriber<Msg> >(nh.get(), "test_topic"));
  c.registerCallback(std::bind(&Helper::cb, &h, _1));
  auto pub = nh->create_publisher<Msg>("test_topic");

  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  while (h.count_ == 0 && (ros_clock.now() - start) < rclcpp::Duration(1.0, 0))
  {
    pub->publish(std::make_shared<Msg>());
    rclcpp::Rate(50).sleep();
    rclcpp::spin_some(nh);
  }

  ASSERT_GT(h.count_, 0);
}

struct ConstHelper
{
  void cb(const MsgConstPtr& msg)
  {
    msg_ = msg;
  }

  MsgConstPtr msg_;
};

struct NonConstHelper
{
  void cb(const MsgPtr& msg)
  {
    msg_ = msg;
  }

  MsgPtr msg_;
};

TEST(Subscriber, singleNonConstCallback)
{
  auto nh = std::make_shared<rclcpp::Node>("test_node");
  NonConstHelper h;
  Subscriber<Msg> sub(nh.get(), "test_topic");
  sub.registerCallback(&NonConstHelper::cb, &h);
  auto pub = nh->create_publisher<Msg>("test_topic");
  pub->publish(std::make_shared<Msg>());

  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(nh);

  ASSERT_TRUE(h.msg_);
//  ASSERT_EQ(msg.get(), h.msg_.get());
}

TEST(Subscriber, multipleNonConstCallbacksFilterSubscriber)
{
  auto nh = std::make_shared<rclcpp::Node>("test_node");
  NonConstHelper h, h2;
  Subscriber<Msg> sub(nh.get(), "test_topic");
  sub.registerCallback(&NonConstHelper::cb, &h);
  sub.registerCallback(&NonConstHelper::cb, &h2);
  auto pub = nh->create_publisher<Msg>("test_topic");
  pub->publish(std::make_shared<Msg>());

  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(nh);

  ASSERT_TRUE(h.msg_);
  ASSERT_TRUE(h2.msg_);
//  EXPECT_NE(msg.get(), h.msg_.get());
//  EXPECT_NE(msg.get(), h2.msg_.get());
//  EXPECT_NE(h.msg_.get(), h2.msg_.get());
}

TEST(Subscriber, multipleCallbacksSomeFilterSomeDirect)
{
  auto nh = std::make_shared<rclcpp::Node>("test_node");
  NonConstHelper h, h2;
  Subscriber<Msg> sub(nh.get(), "test_topic");
  sub.registerCallback(&NonConstHelper::cb, &h);
//  ros::Subscriber sub2 = nh.subscribe("test_topic", 0, &NonConstHelper::cb, &h2);
//  ros::Publisher pub = nh.advertise<Msg>("test_topic", 0);

  auto pub = nh->create_publisher<Msg>("test_topic");
  pub->publish(std::make_shared<Msg>());

  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(nh);

  ASSERT_TRUE(h.msg_);
//  ASSERT_TRUE(h2.msg_);
//  EXPECT_NE(msg.get(), h.msg_.get());
//  EXPECT_NE(msg.get(), h2.msg_.get());
//  EXPECT_NE(h.msg_.get(), h2.msg_.get());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}


