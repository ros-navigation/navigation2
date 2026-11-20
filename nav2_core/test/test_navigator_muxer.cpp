// Copyright (c) 2026 Botronics
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

#include <atomic>
#include <thread>

#include "nav2_core/behavior_tree_navigator.hpp"

// Mock navigator to test NavigatorMuxer class
class MockNavigator : public nav2_core::NavigatorBase
{
public:
  bool on_configure(
    nav2::LifecycleNode::WeakPtr,
    const std::vector<std::string> &,
    const nav2_core::FeedbackUtils &,
    nav2_core::NavigatorMuxer *,
    std::shared_ptr<nav2_util::OdomSmoother>) override {return true;}

  bool on_activate() override {return true;}
  bool on_deactivate() override {return true;}
  bool on_cleanup() override {return true;}

  void preempt() override
  {
    preempt_called_ = true;
  }

  bool preempt_called_ = false;
};

TEST(NavigatorMuxerTest, InitiallyNotNavigating)
{
  nav2_core::NavigatorMuxer muxer;
  EXPECT_FALSE(muxer.isNavigating());
}

TEST(NavigatorMuxerTest, StartNavigatingReturnsTrue)
{
  nav2_core::NavigatorMuxer muxer;
  MockNavigator nav;
  muxer.startNavigating(&nav);
  EXPECT_TRUE(muxer.isNavigating());
}

TEST(NavigatorMuxerTest, StopNavigatingReturnsFalse)
{
  nav2_core::NavigatorMuxer muxer;
  MockNavigator nav;
  muxer.startNavigating(&nav);
  muxer.stopNavigating(&nav);
  EXPECT_FALSE(muxer.isNavigating());
}

TEST(NavigatorMuxerTest, StopWithWrongPointerDoesNotStop)
{
  nav2_core::NavigatorMuxer muxer;
  MockNavigator nav_a, nav_b;
  muxer.startNavigating(&nav_a);
  muxer.stopNavigating(&nav_b);
  EXPECT_TRUE(muxer.isNavigating());
}

TEST(NavigatorMuxerTest, PreemptCurrentNavigatorCallsPreempt)
{
  nav2_core::NavigatorMuxer muxer;
  MockNavigator nav;
  muxer.startNavigating(&nav);
  EXPECT_FALSE(nav.preempt_called_);
  muxer.preemptCurrentNavigator();
  EXPECT_TRUE(nav.preempt_called_);
}

TEST(NavigatorMuxerTest, PreemptWithoutActiveNavigatorIsSafe)
{
  nav2_core::NavigatorMuxer muxer;
  EXPECT_NO_THROW(muxer.preemptCurrentNavigator());
  EXPECT_FALSE(muxer.isNavigating());
}

TEST(NavigatorMuxerTest, StartNavigatingRefusesAnotherOwner)
{
  nav2_core::NavigatorMuxer muxer;
  MockNavigator nav_a, nav_b;
  EXPECT_TRUE(muxer.startNavigating(&nav_a));
  EXPECT_TRUE(muxer.startNavigating(&nav_a));
  EXPECT_FALSE(muxer.startNavigating(&nav_b));
  EXPECT_TRUE(muxer.isCurrentNavigator(&nav_a));
}

TEST(NavigatorMuxerTest, ConcurrentStartNavigatingHasOneOwner)
{
  nav2_core::NavigatorMuxer muxer;
  MockNavigator nav_a, nav_b;
  for (int round = 0; round < 2000; ++round) {
    std::atomic_bool go{false};
    std::atomic_int owners{0};
    auto acquire = [&](nav2_core::NavigatorBase * nav) {
        while (!go) {}
        owners += muxer.startNavigating(nav) ? 1 : 0;
      };
    std::thread first(acquire, &nav_a);
    std::thread second(acquire, &nav_b);
    go = true;
    first.join();
    second.join();
    ASSERT_EQ(owners, 1);
    muxer.stopNavigating(muxer.isCurrentNavigator(&nav_a) ? &nav_a : &nav_b);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
