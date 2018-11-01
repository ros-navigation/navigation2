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
#include <memory>
#include <functional>
#include "message_filters/cache.h"
#include "message_filters/message_traits.h"

using namespace message_filters ;

struct Header
{
  rclcpp::Time stamp ;
} ;


struct Msg
{
  Header header ;
  int data ;
} ;
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

using namespace std ;
void fillCacheEasy(Cache<Msg>& cache, unsigned int start, unsigned int end)

{
  for (unsigned int i=start; i < end; i++)
  {
    Msg* msg = new Msg ;
    msg->data = i ;
    msg->header.stamp= rclcpp::Time(i*10, 0) ;

    std::shared_ptr<Msg const> msg_ptr(msg) ;
    cache.add(msg_ptr) ;
  }
}

TEST(Cache, easyInterval)
{
  Cache<Msg> cache(10) ;
  fillCacheEasy(cache, 0, 5) ;

  vector<std::shared_ptr<Msg const> > interval_data = cache.getInterval(rclcpp::Time(5, 0), rclcpp::Time(35, 0)) ;

  ASSERT_EQ(interval_data.size(), (unsigned int) 3) ;
  EXPECT_EQ(interval_data[0]->data, 1) ;
  EXPECT_EQ(interval_data[1]->data, 2) ;
  EXPECT_EQ(interval_data[2]->data, 3) ;

  // Look for an interval past the end of the cache
  interval_data = cache.getInterval(rclcpp::Time(55, 0), rclcpp::Time(65, 0)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 0) ;

  // Look for an interval that fell off the back of the cache
  fillCacheEasy(cache, 5, 20) ;
  interval_data = cache.getInterval(rclcpp::Time(5, 0), rclcpp::Time(35, 0)) ;
  EXPECT_EQ(interval_data.size(), (unsigned int) 0) ;
}

TEST(Cache, easySurroundingInterval)
{
  Cache<Msg> cache(10);
  fillCacheEasy(cache, 1, 6);

  vector<std::shared_ptr<Msg const> > interval_data;
  interval_data = cache.getSurroundingInterval(rclcpp::Time(15,0), rclcpp::Time(35,0)) ;
  ASSERT_EQ(interval_data.size(), (unsigned int) 4);
  EXPECT_EQ(interval_data[0]->data, 1);
  EXPECT_EQ(interval_data[1]->data, 2);
  EXPECT_EQ(interval_data[2]->data, 3);
  EXPECT_EQ(interval_data[3]->data, 4);

  interval_data = cache.getSurroundingInterval(rclcpp::Time(0,0), rclcpp::Time(35,0)) ;
  ASSERT_EQ(interval_data.size(), (unsigned int) 4);
  EXPECT_EQ(interval_data[0]->data, 1);

  interval_data = cache.getSurroundingInterval(rclcpp::Time(35,0), rclcpp::Time(35,0)) ;
  ASSERT_EQ(interval_data.size(), (unsigned int) 2);
  EXPECT_EQ(interval_data[0]->data, 3);
  EXPECT_EQ(interval_data[1]->data, 4);

  interval_data = cache.getSurroundingInterval(rclcpp::Time(55,0), rclcpp::Time(55,0)) ;
  ASSERT_EQ(interval_data.size(), (unsigned int) 1);
  EXPECT_EQ(interval_data[0]->data, 5);
}


std::shared_ptr<Msg const> buildMsg(double time, int data)
{
  Msg* msg = new Msg ;
  msg->data = data ;
  msg->header.stamp = rclcpp::Time(time, 0) ;

  std::shared_ptr<Msg const> msg_ptr(msg) ;
  return msg_ptr ;
}

TEST(Cache, easyUnsorted)
{
  Cache<Msg> cache(10) ;

  cache.add(buildMsg(10.0, 1)) ;
  cache.add(buildMsg(30.0, 3)) ;
  cache.add(buildMsg(70.0, 7)) ;
  cache.add(buildMsg( 5.0, 0)) ;
  cache.add(buildMsg(20.0, 2)) ;

  vector<std::shared_ptr<Msg const> > interval_data = cache.getInterval(rclcpp::Time(3, 0), rclcpp::Time(15, 0)) ;

  ASSERT_EQ(interval_data.size(), (unsigned int) 2) ;
  EXPECT_EQ(interval_data[0]->data, 0) ;
  EXPECT_EQ(interval_data[1]->data, 1) ;

  // Grab all the data
  interval_data = cache.getInterval(rclcpp::Time(0, 0), rclcpp::Time(80, 0)) ;
  ASSERT_EQ(interval_data.size(), (unsigned int) 5) ;
  EXPECT_EQ(interval_data[0]->data, 0) ;
  EXPECT_EQ(interval_data[1]->data, 1) ;
  EXPECT_EQ(interval_data[2]->data, 2) ;
  EXPECT_EQ(interval_data[3]->data, 3) ;
  EXPECT_EQ(interval_data[4]->data, 7) ;
}


TEST(Cache, easyElemBeforeAfter)
{
  Cache<Msg> cache(10) ;
  std::shared_ptr<Msg const> elem_ptr ;

  fillCacheEasy(cache, 5, 10) ;

  elem_ptr = cache.getElemAfterTime( rclcpp::Time(85.0, 0)) ;

  ASSERT_FALSE(!elem_ptr) ;
  EXPECT_EQ(elem_ptr->data, 9) ;

  elem_ptr = cache.getElemBeforeTime( rclcpp::Time(85.0, 0)) ;
  ASSERT_FALSE(!elem_ptr) ;
  EXPECT_EQ(elem_ptr->data, 8) ;

  elem_ptr = cache.getElemBeforeTime( rclcpp::Time(45.0, 0)) ;
  EXPECT_TRUE(!elem_ptr) ;
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

TEST(Cache, eventInEventOut)
{
  Cache<Msg> c0(10);
  Cache<Msg> c1(c0, 10);
  EventHelper h;
  c1.registerCallback(&EventHelper::cb, &h);

  MessageEvent<Msg const> evt(std::make_shared<Msg const>(), rclcpp::Time(4, 0));
  c0.add(evt);

  EXPECT_EQ(h.event_.getReceiptTime(), evt.getReceiptTime());
  EXPECT_EQ(h.event_.getMessage(), evt.getMessage());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}

