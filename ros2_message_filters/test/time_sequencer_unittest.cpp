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
#include "message_filters/time_sequencer.h"

using namespace message_filters;

struct Header
{
  rclcpp::Time stamp;
};


struct Msg
{
  Header header;
  int data;
};
typedef std::shared_ptr<Msg> MsgPtr;
typedef std::shared_ptr<Msg const> MsgConstPtr;

namespace message_filters
{
namespace message_traits
{
template<>
struct TimeStamp<Msg>
{
  static rclcpp::Time value(const Msg& m)
  {
    return m.header.stamp;
  }
};
}
}

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

TEST(TimeSequencer, simple)
{
  TimeSequencer<Msg> seq(rclcpp::Duration(1, 0), rclcpp::Duration(0, 10000000), 10);
  Helper h;
  seq.registerCallback(std::bind(&Helper::cb, &h, _1));
  MsgPtr msg(std::make_shared<Msg>());
  msg->header.stamp = rclcpp::Clock().now();
  seq.add(msg);

  rclcpp::Rate(10).sleep();
  rclcpp::spin_some(seq.get_node());
  ASSERT_EQ(h.count_, 0);

//  ros::Time::setNow(rclcpp::Clock(RCL_ROS_TIME)::now() + rclcpp::Duration(2, 0));

  rclcpp::Rate(1).sleep();
  rclcpp::spin_some(seq.get_node());

  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSequencer, compilation)
{
  TimeSequencer<Msg> seq(rclcpp::Duration(1, 0), rclcpp::Duration(0, 10000000), 10);
  TimeSequencer<Msg> seq2(rclcpp::Duration(1, 0), rclcpp::Duration(0, 10000000), 10);
  seq2.connectInput(seq);
}

struct EventHelper
{
public:
  void cb(const MessageEvent<Msg const>& evt)
  {
    event_ = evt;
  }

  MessageEvent<Msg const> event_;
};
TEST(TimeSequencer, eventInEventOut)
{
  rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("test_node");
  TimeSequencer<Msg> seq(rclcpp::Duration(1, 0), rclcpp::Duration(0, 10000000), 10, nh);
  TimeSequencer<Msg> seq2(seq, rclcpp::Duration(1, 0), rclcpp::Duration(0, 10000000), 10, nh);
  EventHelper h;
  seq2.registerCallback(&EventHelper::cb, &h);

  MessageEvent<Msg const> evt(std::make_shared<Msg const>(), rclcpp::Clock().now());
  seq.add(evt);

//  ros::Time::setNow(ros::Time::now() + ros::Duration(2));
  rclcpp::Rate(0.5).sleep();
  while (!h.event_.getMessage())
  {
    rclcpp::Rate(100).sleep();
    rclcpp::spin_some(nh);
  }

  EXPECT_EQ(h.event_.getReceiptTime(), evt.getReceiptTime());
  EXPECT_EQ(h.event_.getMessage(), evt.getMessage());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}


