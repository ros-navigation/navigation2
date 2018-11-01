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
#include "message_filters/chain.h"

using namespace message_filters;

struct Msg
{
};
typedef std::shared_ptr<Msg> MsgPtr;
typedef std::shared_ptr<Msg const> MsgConstPtr;

class Helper
{
public:
  Helper()
  : count_(0)
  {}

  void cb()
  {
    ++count_;
  }

  int32_t count_;
};

typedef std::shared_ptr<PassThrough<Msg> > PassThroughPtr;

TEST(Chain, simple)
{
  Helper h;
  Chain<Msg> c;
  c.addFilter(std::make_shared<PassThrough<Msg> >());
  c.registerCallback(std::bind(&Helper::cb, &h));

  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 1);
  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 2);
}

TEST(Chain, multipleFilters)
{
  Helper h;
  Chain<Msg> c;
  c.addFilter(std::make_shared<PassThrough<Msg> >());
  c.addFilter(std::make_shared<PassThrough<Msg> >());
  c.addFilter(std::make_shared<PassThrough<Msg> >());
  c.addFilter(std::make_shared<PassThrough<Msg> >());
  c.registerCallback(std::bind(&Helper::cb, &h));

  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 1);
  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 2);
}

TEST(Chain, addingFilters)
{
  Helper h;
  Chain<Msg> c;
  c.addFilter(std::make_shared<PassThrough<Msg> >());
  c.addFilter(std::make_shared<PassThrough<Msg> >());
  c.registerCallback(std::bind(&Helper::cb, &h));

  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 1);

  c.addFilter(std::make_shared<PassThrough<Msg> >());
  c.addFilter(std::make_shared<PassThrough<Msg> >());

  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 2);
}

TEST(Chain, inputFilter)
{
  Helper h;
  Chain<Msg> c;
  c.addFilter(std::make_shared<PassThrough<Msg> >());
  c.registerCallback(std::bind(&Helper::cb, &h));

  PassThrough<Msg> p;
  c.connectInput(p);
  p.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 1);

  p.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 2);
}

TEST(Chain, nonSharedPtrFilter)
{
  Helper h;
  Chain<Msg> c;
  PassThrough<Msg> p;
  c.addFilter(&p);
  c.registerCallback(std::bind(&Helper::cb, &h));

  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 1);
  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 2);
}

TEST(Chain, retrieveFilter)
{
  Chain<Msg> c;

  ASSERT_FALSE(c.getFilter<PassThrough<Msg> >(0));

  c.addFilter(std::make_shared<PassThrough<Msg> >());

  ASSERT_TRUE(c.getFilter<PassThrough<Msg> >(0));
  ASSERT_FALSE(c.getFilter<PassThrough<Msg> >(1));
}

TEST(Chain, retrieveFilterThroughBaseClass)
{
  Chain<Msg> c;
  ChainBase* cb = &c;

  ASSERT_FALSE(cb->getFilter<PassThrough<Msg> >(0));

  c.addFilter(std::make_shared<PassThrough<Msg> >());

  ASSERT_TRUE(cb->getFilter<PassThrough<Msg> >(0));
  ASSERT_FALSE(cb->getFilter<PassThrough<Msg> >(1));
}

struct PTDerived : public PassThrough<Msg>
{

};

TEST(Chain, retrieveBaseClass)
{
  Chain<Msg> c;
  c.addFilter(std::make_shared<PTDerived>());
  ASSERT_TRUE(c.getFilter<PassThrough<Msg> >(0));
  ASSERT_TRUE(c.getFilter<PTDerived>(0));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}


