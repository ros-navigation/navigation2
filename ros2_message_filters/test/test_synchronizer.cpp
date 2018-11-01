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
#include "message_filters/synchronizer.h"
#include <array>

using namespace message_filters;
using namespace std::placeholders;

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


template<typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
         typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
struct NullPolicy : public PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
  typedef Synchronizer<NullPolicy> Sync;
  typedef PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8> Super;
  typedef typename Super::Messages Messages;
  typedef typename Super::Signal Signal;
  typedef typename Super::Events Events;
  typedef typename Super::RealTypeCount RealTypeCount;

  NullPolicy()
  {
    for (int i = 0; i < RealTypeCount::value; ++i)
    {
      added_[i] = 0;
    }
  }

  void initParent(Sync*)
  {
  }

  template<int i>
  void add(const typename std::tuple_element<i, Events>::type&)
  {
    ++added_.at(i);
  }

  std::array<int32_t, RealTypeCount::value> added_;
};
typedef NullPolicy<Msg, Msg> Policy2;
typedef NullPolicy<Msg, Msg, Msg> Policy3;
typedef NullPolicy<Msg, Msg, Msg, Msg> Policy4;
typedef NullPolicy<Msg, Msg, Msg, Msg, Msg> Policy5;
typedef NullPolicy<Msg, Msg, Msg, Msg, Msg, Msg> Policy6;
typedef NullPolicy<Msg, Msg, Msg, Msg, Msg, Msg, Msg> Policy7;
typedef NullPolicy<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> Policy8;
typedef NullPolicy<Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg, Msg> Policy9;

TEST(Synchronizer, compile2)
{
  NullFilter<Msg> f0, f1;
  Synchronizer<Policy2> sync(f0, f1);
}

TEST(Synchronizer, compile3)
{
  NullFilter<Msg> f0, f1, f2;
  Synchronizer<Policy3> sync(f0, f1, f2);
}

TEST(Synchronizer, compile4)
{
  NullFilter<Msg> f0, f1, f2, f3;
  Synchronizer<Policy4> sync(f0, f1, f2, f3);
}

TEST(Synchronizer, compile5)
{
  NullFilter<Msg> f0, f1, f2, f3, f4;
  Synchronizer<Policy5> sync(f0, f1, f2, f3, f4);
}

TEST(Synchronizer, compile6)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5;
  Synchronizer<Policy6> sync(f0, f1, f2, f3, f4, f5);
}

TEST(Synchronizer, compile7)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5, f6;
  Synchronizer<Policy7> sync(f0, f1, f2, f3, f4, f5, f6);
}

TEST(Synchronizer, compile8)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5, f6, f7;
  Synchronizer<Policy8> sync(f0, f1, f2, f3, f4, f5, f6, f7);
}

TEST(Synchronizer, compile9)
{
  NullFilter<Msg> f0, f1, f2, f3, f4, f5, f6, f7, f8;
  Synchronizer<Policy9> sync(f0, f1, f2, f3, f4, f5, f6, f7, f8);
}

void function2(const MsgConstPtr&, const MsgConstPtr&) {}
void function3(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function4(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function5(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function6(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function7(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function8(const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&, const MsgConstPtr&) {}
void function9(const MsgConstPtr&, MsgConstPtr, const MsgPtr&, MsgPtr, const Msg&, Msg, const MessageEvent<Msg const>&, const MessageEvent<Msg>&, const MsgConstPtr&) {}

TEST(Synchronizer, compileFunction2)
{
  Synchronizer<Policy2> sync;
  sync.registerCallback(function2);
}

TEST(Synchronizer, compileFunction3)
{
  Synchronizer<Policy3> sync;
  sync.registerCallback(function3);
}

TEST(Synchronizer, compileFunction4)
{
  Synchronizer<Policy4> sync;
  sync.registerCallback(function4);
}

TEST(Synchronizer, compileFunction5)
{
  Synchronizer<Policy5> sync;
  sync.registerCallback(function5);
}

TEST(Synchronizer, compileFunction6)
{
  Synchronizer<Policy6> sync;
  sync.registerCallback(function6);
}

TEST(Synchronizer, compileFunction7)
{
  Synchronizer<Policy7> sync;
  sync.registerCallback(function7);
}

TEST(Synchronizer, compileFunction8)
{
  Synchronizer<Policy8> sync;
  sync.registerCallback(function8);
}

TEST(Synchronizer, compileFunction9)
{
  Synchronizer<Policy9> sync;
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

TEST(Synchronizer, compileMethod2)
{
  MethodHelper h;
  Synchronizer<Policy2> sync;
  sync.registerCallback(&MethodHelper::method2, &h);
}

TEST(Synchronizer, compileMethod3)
{
  MethodHelper h;
  Synchronizer<Policy3> sync;
  sync.registerCallback(&MethodHelper::method3, &h);
}

TEST(Synchronizer, compileMethod4)
{
  MethodHelper h;
  Synchronizer<Policy4> sync;
  sync.registerCallback(&MethodHelper::method4, &h);
}

TEST(Synchronizer, compileMethod5)
{
  MethodHelper h;
  Synchronizer<Policy5> sync;
  sync.registerCallback(&MethodHelper::method5, &h);
}

TEST(Synchronizer, compileMethod6)
{
  MethodHelper h;
  Synchronizer<Policy6> sync;
  sync.registerCallback(&MethodHelper::method6, &h);
}

TEST(Synchronizer, compileMethod7)
{
  MethodHelper h;
  Synchronizer<Policy7> sync;
  sync.registerCallback(&MethodHelper::method7, &h);
}

TEST(Synchronizer, compileMethod8)
{
  MethodHelper h;
  Synchronizer<Policy8> sync;
  sync.registerCallback(&MethodHelper::method8, &h);
}

TEST(Synchronizer, add2)
{
  Synchronizer<Policy2> sync;
  MsgPtr m(std::make_shared<Msg>());

  ASSERT_EQ(sync.added_[0], 0);
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  ASSERT_EQ(sync.added_[1], 0);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
}

TEST(Synchronizer, add3)
{
  Synchronizer<Policy3> sync;
  MsgPtr m(std::make_shared<Msg>());

  ASSERT_EQ(sync.added_[0], 0);
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  ASSERT_EQ(sync.added_[1], 0);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  ASSERT_EQ(sync.added_[2], 0);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
}

TEST(Synchronizer, add4)
{
  Synchronizer<Policy4> sync;
  MsgPtr m(std::make_shared<Msg>());

  ASSERT_EQ(sync.added_[0], 0);
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  ASSERT_EQ(sync.added_[1], 0);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  ASSERT_EQ(sync.added_[2], 0);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
  ASSERT_EQ(sync.added_[3], 0);
  sync.add<3>(m);
  ASSERT_EQ(sync.added_[3], 1);
}

TEST(Synchronizer, add5)
{
  Synchronizer<Policy5> sync;
  MsgPtr m(std::make_shared<Msg>());

  ASSERT_EQ(sync.added_[0], 0);
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  ASSERT_EQ(sync.added_[1], 0);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  ASSERT_EQ(sync.added_[2], 0);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
  ASSERT_EQ(sync.added_[3], 0);
  sync.add<3>(m);
  ASSERT_EQ(sync.added_[3], 1);
  ASSERT_EQ(sync.added_[4], 0);
  sync.add<4>(m);
  ASSERT_EQ(sync.added_[4], 1);
}

TEST(Synchronizer, add6)
{
  Synchronizer<Policy6> sync;
  MsgPtr m(std::make_shared<Msg>());

  ASSERT_EQ(sync.added_[0], 0);
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  ASSERT_EQ(sync.added_[1], 0);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  ASSERT_EQ(sync.added_[2], 0);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
  ASSERT_EQ(sync.added_[3], 0);
  sync.add<3>(m);
  ASSERT_EQ(sync.added_[3], 1);
  ASSERT_EQ(sync.added_[4], 0);
  sync.add<4>(m);
  ASSERT_EQ(sync.added_[4], 1);
  ASSERT_EQ(sync.added_[5], 0);
  sync.add<5>(m);
  ASSERT_EQ(sync.added_[5], 1);
}

TEST(Synchronizer, add7)
{
  Synchronizer<Policy7> sync;
  MsgPtr m(std::make_shared<Msg>());

  ASSERT_EQ(sync.added_[0], 0);
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  ASSERT_EQ(sync.added_[1], 0);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  ASSERT_EQ(sync.added_[2], 0);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
  ASSERT_EQ(sync.added_[3], 0);
  sync.add<3>(m);
  ASSERT_EQ(sync.added_[3], 1);
  ASSERT_EQ(sync.added_[4], 0);
  sync.add<4>(m);
  ASSERT_EQ(sync.added_[4], 1);
  ASSERT_EQ(sync.added_[5], 0);
  sync.add<5>(m);
  ASSERT_EQ(sync.added_[5], 1);
  ASSERT_EQ(sync.added_[6], 0);
  sync.add<6>(m);
  ASSERT_EQ(sync.added_[6], 1);
}

TEST(Synchronizer, add8)
{
  Synchronizer<Policy8> sync;
  MsgPtr m(std::make_shared<Msg>());

  ASSERT_EQ(sync.added_[0], 0);
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  ASSERT_EQ(sync.added_[1], 0);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  ASSERT_EQ(sync.added_[2], 0);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
  ASSERT_EQ(sync.added_[3], 0);
  sync.add<3>(m);
  ASSERT_EQ(sync.added_[3], 1);
  ASSERT_EQ(sync.added_[4], 0);
  sync.add<4>(m);
  ASSERT_EQ(sync.added_[4], 1);
  ASSERT_EQ(sync.added_[5], 0);
  sync.add<5>(m);
  ASSERT_EQ(sync.added_[5], 1);
  ASSERT_EQ(sync.added_[6], 0);
  sync.add<6>(m);
  ASSERT_EQ(sync.added_[6], 1);
  ASSERT_EQ(sync.added_[7], 0);
  sync.add<7>(m);
  ASSERT_EQ(sync.added_[7], 1);
}

TEST(Synchronizer, add9)
{
  Synchronizer<Policy9> sync;
  MsgPtr m(std::make_shared<Msg>());

  ASSERT_EQ(sync.added_[0], 0);
  sync.add<0>(m);
  ASSERT_EQ(sync.added_[0], 1);
  ASSERT_EQ(sync.added_[1], 0);
  sync.add<1>(m);
  ASSERT_EQ(sync.added_[1], 1);
  ASSERT_EQ(sync.added_[2], 0);
  sync.add<2>(m);
  ASSERT_EQ(sync.added_[2], 1);
  ASSERT_EQ(sync.added_[3], 0);
  sync.add<3>(m);
  ASSERT_EQ(sync.added_[3], 1);
  ASSERT_EQ(sync.added_[4], 0);
  sync.add<4>(m);
  ASSERT_EQ(sync.added_[4], 1);
  ASSERT_EQ(sync.added_[5], 0);
  sync.add<5>(m);
  ASSERT_EQ(sync.added_[5], 1);
  ASSERT_EQ(sync.added_[6], 0);
  sync.add<6>(m);
  ASSERT_EQ(sync.added_[6], 1);
  ASSERT_EQ(sync.added_[7], 0);
  sync.add<7>(m);
  ASSERT_EQ(sync.added_[7], 1);
  ASSERT_EQ(sync.added_[8], 0);
  sync.add<8>(m);
  ASSERT_EQ(sync.added_[8], 1);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "blah");
//
//  ros::Time::init();
//  ros::Time::setNow(ros::Time());
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}


