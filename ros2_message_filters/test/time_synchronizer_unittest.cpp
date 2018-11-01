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

#include "message_filters/time_synchronizer.h"
#include "message_filters/pass_through.h"
#include "message_filters/message_traits.h"
#include <rclcpp/rclcpp.hpp>

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
  , drop_count_(0)
  {}

  void cb()
  {
    ++count_;
  }

  void dropcb()
  {
    ++drop_count_;
  }

  int32_t count_;
  int32_t drop_count_;
};

TEST(TimeSynchronizer, compile2)
{
  NullFilter<Msg> f0, f1;
  TimeSynchronizer<Msg, Msg> sync(f0, f1, 1);
}

TEST(TimeSynchronizer, compile3)
{
  NullFilter<Msg> f0, f1, f2;
  TimeSynchronizer<Msg, Msg, Msg> sync(f0, f1, f2, 1);
}

TEST(TimeSynchronizer, compile4)
{
  NullFilter<Msg> f0, f1, f2, f3;
  TimeSynchronizer<Msg, Msg, Msg, Msg> sync(f0, f1, f2, f3, 1);
}

TEST(TimeSynchronizer, compile5)
{
  NullFilter<Msg> f0, f1, f2, f3, f4;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg> sync(f0, f1, f2, f3, f4, 1);
}

TEST(TimeSynchronizer, compile6)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg> sync(f0, f1, f2, f3, f4, f5, 1);
}

TEST(TimeSynchronizer, compile7)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5, f6;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(f0, f1, f2, f3, f4, f5, f6, 1);
}

TEST(TimeSynchronizer, compile8)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5, f6, f7;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(f0, f1, f2, f3, f4, f5, f6, f7, 1);
}

TEST(TimeSynchronizer, compile9)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5, f6, f7, f8;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(f0, f1, f2, f3, f4, f5, f6, f7, f8, 1);
}

void function2(const MsgConstPtr&, const MsgConstPtr&) {}
void function3(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function4(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function5(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function6(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function7(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function8(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function9(const MsgConstPtr&, MsgConstPtr, const MsgPtr&, MsgPtr, const Msg&, Msg, const MessageEvent<Msg const>&, const MessageEvent<Msg>&, const MsgConstPtr&) {}

TEST(TimeSynchronizer, compileFunction2)
{
  TimeSynchronizer<Msg, Msg> sync(1);
  sync.registerCallback(function2);
}

TEST(TimeSynchronizer, compileFunction3)
{
  TimeSynchronizer<Msg, Msg, Msg> sync(1);
  sync.registerCallback(function3);
}

TEST(TimeSynchronizer, compileFunction4)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(function4);
}

TEST(TimeSynchronizer, compileFunction5)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(function5);
}

TEST(TimeSynchronizer, compileFunction6)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(function6);
}

TEST(TimeSynchronizer, compileFunction7)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(function7);
}

TEST(TimeSynchronizer, compileFunction8)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(function8);
}

TEST(TimeSynchronizer, compileFunction9)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(function9);
}

struct MethodHelper
{
  void method2(const MsgConstPtr&, const MsgConstPtr&) {}
  void method3(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
  void method4(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
  void method5(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
  void method6(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
  void method7(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
  void method8(const MsgConstPtr&, MsgConstPtr, const MsgPtr&, MsgPtr, const Msg&, Msg, const MessageEvent<Msg const>&, const MessageEvent<Msg>&) {}
  // Can only do 8 here because the object instance counts as a parameter and bind only supports 9
};

TEST(TimeSynchronizer, compileMethod2)
{
  MethodHelper h;
  TimeSynchronizer<Msg, Msg> sync(1);
  sync.registerCallback(&MethodHelper::method2, &h);
}

TEST(TimeSynchronizer, compileMethod3)
{
  MethodHelper h;
  TimeSynchronizer<Msg, Msg, Msg> sync(1);
  sync.registerCallback(&MethodHelper::method3, &h);
}

TEST(TimeSynchronizer, compileMethod4)
{
  MethodHelper h;
  TimeSynchronizer<Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(&MethodHelper::method4, &h);
}

TEST(TimeSynchronizer, compileMethod5)
{
  MethodHelper h;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(&MethodHelper::method5, &h);
}

TEST(TimeSynchronizer, compileMethod6)
{
  MethodHelper h;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(&MethodHelper::method6, &h);
}

TEST(TimeSynchronizer, compileMethod7)
{
  MethodHelper h;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(&MethodHelper::method7, &h);
}

TEST(TimeSynchronizer, compileMethod8)
{
  MethodHelper h;
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  sync.registerCallback(&MethodHelper::method8, &h);
}

TEST(TimeSynchronizer, immediate2)
{
  TimeSynchronizer<Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->header.stamp = rclcpp::Clock().now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate3)
{
  TimeSynchronizer<Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->header.stamp = rclcpp::Clock().now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate4)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->header.stamp = rclcpp::Clock().now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
  sync.add3(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate5)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->header.stamp = rclcpp::Clock().now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
  sync.add3(m);
  ASSERT_EQ(h.count_, 0);
  sync.add4(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate6)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->header.stamp = rclcpp::Clock().now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
  sync.add3(m);
  ASSERT_EQ(h.count_, 0);
  sync.add4(m);
  ASSERT_EQ(h.count_, 0);
  sync.add5(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate7)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->header.stamp = rclcpp::Clock().now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
  sync.add3(m);
  ASSERT_EQ(h.count_, 0);
  sync.add4(m);
  ASSERT_EQ(h.count_, 0);
  sync.add5(m);
  ASSERT_EQ(h.count_, 0);
  sync.add6(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate8)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->header.stamp = rclcpp::Clock().now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
  sync.add3(m);
  ASSERT_EQ(h.count_, 0);
  sync.add4(m);
  ASSERT_EQ(h.count_, 0);
  sync.add5(m);
  ASSERT_EQ(h.count_, 0);
  sync.add6(m);
  ASSERT_EQ(h.count_, 0);
  sync.add7(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, immediate9)
{
  TimeSynchronizer<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->header.stamp = rclcpp::Clock().now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
  sync.add3(m);
  ASSERT_EQ(h.count_, 0);
  sync.add4(m);
  ASSERT_EQ(h.count_, 0);
  sync.add5(m);
  ASSERT_EQ(h.count_, 0);
  sync.add6(m);
  ASSERT_EQ(h.count_, 0);
  sync.add7(m);
  ASSERT_EQ(h.count_, 0);
  sync.add8(m);
  ASSERT_EQ(h.count_, 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// From here on we assume that testing the 3-message version is sufficient, so as not to duplicate
// tests for everywhere from 2-9
//////////////////////////////////////////////////////////////////////////////////////////////////
TEST(TimeSynchronizer, multipleTimes)
{
  TimeSynchronizer<Msg, Msg, Msg> sync(2);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->header.stamp = rclcpp::Time();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);

  m = std::make_shared<Msg>();
  m->header.stamp = rclcpp::Time(100000000);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, queueSize)
{
  TimeSynchronizer<Msg, Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->header.stamp = rclcpp::Time();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);

  m = std::make_shared<Msg>();
  m->header.stamp = rclcpp::Time(100000000);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);

  m = std::make_shared<Msg>();
  m->header.stamp = rclcpp::Time();
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add2(m);
  ASSERT_EQ(h.count_, 0);
}

TEST(TimeSynchronizer, dropCallback)
{
  TimeSynchronizer<Msg, Msg> sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  sync.registerDropCallback(std::bind(&Helper::dropcb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->header.stamp = rclcpp::Time(0, 0);

  sync.add0(m);
  ASSERT_EQ(h.drop_count_, 0);
  m->header.stamp = rclcpp::Time(100000000);
  sync.add0(m);

  ASSERT_EQ(h.drop_count_, 1);
}

struct EventHelper
{
  void callback(const MessageEvent<Msg const>& e1, const MessageEvent<Msg const>& e2)
  {
    e1_ = e1;
    e2_ = e2;
  }

  MessageEvent<Msg const> e1_;
  MessageEvent<Msg const> e2_;
};

TEST(TimeSynchronizer, eventInEventOut)
{
  TimeSynchronizer<Msg, Msg> sync(2);
  EventHelper h;
  sync.registerCallback(&EventHelper::callback, &h);
  MessageEvent<Msg const> evt(std::make_shared<Msg>(), rclcpp::Time(4, 0));

  sync.add<0>(evt);
  sync.add<1>(evt);

  ASSERT_TRUE(h.e1_.getMessage());
  ASSERT_TRUE(h.e2_.getMessage());
  ASSERT_EQ(h.e1_.getReceiptTime(), evt.getReceiptTime());
  ASSERT_EQ(h.e2_.getReceiptTime(), evt.getReceiptTime());
}

TEST(TimeSynchronizer, connectConstructor)
{
  PassThrough<Msg> pt1, pt2;
  TimeSynchronizer<Msg, Msg> sync(pt1, pt2, 1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->header.stamp = rclcpp::Clock().now();

  pt1.add(m);
  ASSERT_EQ(h.count_, 0);
  pt2.add(m);
  ASSERT_EQ(h.count_, 1);
}

//TEST(TimeSynchronizer, connectToSimple)

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}



