/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
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
#include <functional>
#include <memory>
#include <array>
#include "message_filters/simple_filter.h"

using namespace message_filters;
using namespace std::placeholders;

struct Msg
{
};
typedef std::shared_ptr<Msg> MsgPtr;
typedef std::shared_ptr<Msg const> MsgConstPtr;

struct Filter : public SimpleFilter<Msg>
{
  typedef MessageEvent<Msg const> EventType;

  void add(const EventType& evt)
  {
    signalMessage(evt);
  }
};

class Helper
{
public:
  Helper()
  {
    counts_.fill(0);
  }

  void cb0(const MsgConstPtr&)
  {
    ++counts_[0];
  }

  void cb1(const Msg&)
  {
    ++counts_[1];
  }

  void cb2(MsgConstPtr)
  {
    ++counts_[2];
  }

  void cb3(const MessageEvent<Msg const>&)
  {
    ++counts_[3];
  }

  void cb4(Msg)
  {
    ++counts_[4];
  }

  void cb5(const MsgPtr&)
  {
    ++counts_[5];
  }

  void cb6(MsgPtr)
  {
    ++counts_[6];
  }

  void cb7(const MessageEvent<Msg>&)
  {
    ++counts_[7];
  }

  std::array<int32_t, 30> counts_;
};

TEST(SimpleFilter, callbackTypes)
{
  Helper h;
  Filter f;
  f.registerCallback(std::bind(&Helper::cb0, &h, _1));
  f.registerCallback<const Msg&>(std::bind(&Helper::cb1, &h, _1));
  f.registerCallback<MsgConstPtr>(std::bind(&Helper::cb2, &h, _1));
  f.registerCallback<const MessageEvent<Msg const>&>(std::bind(&Helper::cb3, &h, _1));
  f.registerCallback<Msg>(std::bind(&Helper::cb4, &h, _1));
  f.registerCallback<const MsgPtr&>(std::bind(&Helper::cb5, &h, _1));
  f.registerCallback<MsgPtr>(std::bind(&Helper::cb6, &h, _1));
  f.registerCallback<const MessageEvent<Msg>&>(std::bind(&Helper::cb7, &h, _1));

  f.add(Filter::EventType(std::make_shared<Msg>()));
  EXPECT_EQ(h.counts_[0], 1);
  EXPECT_EQ(h.counts_[1], 1);
  EXPECT_EQ(h.counts_[2], 1);
  EXPECT_EQ(h.counts_[3], 1);
  EXPECT_EQ(h.counts_[4], 1);
  EXPECT_EQ(h.counts_[5], 1);
  EXPECT_EQ(h.counts_[6], 1);
  EXPECT_EQ(h.counts_[7], 1);
}

struct OldFilter
{
  Connection registerCallback(const std::function<void(const MsgConstPtr&)>&)
  {
    return Connection();
  }
};

TEST(SimpleFilter, oldRegisterWithNewFilter)
{
  OldFilter f;
  Helper h;
  f.registerCallback(std::bind(&Helper::cb3, &h, _1));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}



