
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
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/message_traits.h"
#include <vector>

using namespace message_filters;
using namespace message_filters::sync_policies;

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

typedef std::pair<rclcpp::Time, rclcpp::Time> TimePair;
typedef std::pair<rclcpp::Time, unsigned int> TimeAndTopic;
struct TimeQuad
{
  TimeQuad(rclcpp::Time p, rclcpp::Time q, rclcpp::Time r, rclcpp::Time s)
  {
    time[0] = p;
    time[1] = q;
    time[2] = r;
    time[3] = s;
  }
  rclcpp::Time time[4];
};


//----------------------------------------------------------
//                Test Class (for 2 inputs)
//----------------------------------------------------------
class ApproximateTimeSynchronizerTest
{
public:

  ApproximateTimeSynchronizerTest(const std::vector<TimeAndTopic> &input,
				  const std::vector<TimePair> &output,
				  uint32_t queue_size) :
    input_(input), output_(output), output_position_(0), sync_(queue_size)
  {
    sync_.registerCallback(std::bind(&ApproximateTimeSynchronizerTest::callback, this, _1, _2));
  }

  void callback(const MsgConstPtr& p, const MsgConstPtr& q)
  {
    //printf("Call_back called\n");
    //printf("Call back: <%f, %f>\n", p->header.stamp.toSec(), q->header.stamp.toSec());
    ASSERT_TRUE(p);
    ASSERT_TRUE(q);
    ASSERT_LT(output_position_, output_.size());
    EXPECT_EQ(output_[output_position_].first, p->header.stamp);
    EXPECT_EQ(output_[output_position_].second, q->header.stamp);
    ++output_position_;
  }

  void run()
  {
    for (unsigned int i = 0; i < input_.size(); i++)
    {
      if (input_[i].second == 0)
      {
        MsgPtr p(std::make_shared<Msg>());
        p->header.stamp = input_[i].first;
        sync_.add<0>(p);
      }
      else
      {
        MsgPtr q(std::make_shared<Msg>());
        q->header.stamp = input_[i].first;
        sync_.add<1>(q);
      }
    }
    //printf("Done running test\n");
    EXPECT_EQ(output_.size(), output_position_);
  }

private:
  const std::vector<TimeAndTopic> &input_;
  const std::vector<TimePair> &output_;
  unsigned int output_position_;
  typedef Synchronizer<ApproximateTime<Msg, Msg> > Sync2;
public:
  Sync2 sync_;
};


//----------------------------------------------------------
//                Test Class (for 4 inputs)
//----------------------------------------------------------
class ApproximateTimeSynchronizerTestQuad
{
public:

  ApproximateTimeSynchronizerTestQuad(const std::vector<TimeAndTopic> &input,
				      const std::vector<TimeQuad> &output,
				      uint32_t queue_size) :
    input_(input), output_(output), output_position_(0), sync_(queue_size)
  {
    sync_.registerCallback(std::bind(&ApproximateTimeSynchronizerTestQuad::callback, this, _1, _2, _3, _4));
  }

    void callback(const MsgConstPtr& p, const MsgConstPtr& q, const MsgConstPtr& r, const MsgConstPtr& s)
  {
    //printf("Call_back called\n");
    //printf("Call back: <%f, %f>\n", p->header.stamp.toSec(), q->header.stamp.toSec());
    ASSERT_TRUE(p);
    ASSERT_TRUE(q);
    ASSERT_TRUE(r);
    ASSERT_TRUE(s);
    ASSERT_LT(output_position_, output_.size());
    EXPECT_EQ(output_[output_position_].time[0], p->header.stamp);
    EXPECT_EQ(output_[output_position_].time[1], q->header.stamp);
    EXPECT_EQ(output_[output_position_].time[2], r->header.stamp);
    EXPECT_EQ(output_[output_position_].time[3], s->header.stamp);
    ++output_position_;
  }

  void run()
  {
    for (unsigned int i = 0; i < input_.size(); i++)
    {
      MsgPtr p(std::make_shared<Msg>());
      p->header.stamp = input_[i].first;
      switch (input_[i].second)
      {
        case 0:
          sync_.add<0>(p);
          break;
        case 1:
          sync_.add<1>(p);
          break;
        case 2:
          sync_.add<2>(p);
          break;
        case 3:
          sync_.add<3>(p);
          break;
      }
    }
    //printf("Done running test\n");
    EXPECT_EQ(output_.size(), output_position_);
  }

private:
  const std::vector<TimeAndTopic> &input_;
  const std::vector<TimeQuad> &output_;
  unsigned int output_position_;
  typedef Synchronizer<ApproximateTime<Msg, Msg, Msg, Msg> > Sync4;
public:
  Sync4 sync_;
};


//----------------------------------------------------------
//                   Test Suite
//----------------------------------------------------------
TEST(ApproxTimeSync, ExactMatch) {
  // Input A:  a..b..c
  // Input B:  A..B..C
  // Output:   a..b..c
  //           A..B..C
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t,1));   // A
  input.push_back(TimeAndTopic(t+s*3,0)); // b
  input.push_back(TimeAndTopic(t+s*3,1)); // B
  input.push_back(TimeAndTopic(t+s*6,0)); // c
  input.push_back(TimeAndTopic(t+s*6,1)); // C
  output.push_back(TimePair(t, t));
  output.push_back(TimePair(t+s*3, t+s*3));
  output.push_back(TimePair(t+s*6, t+s*6));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10);
  sync_test.run();
}


TEST(ApproxTimeSync, PerfectMatch) {
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   ...a..b.
  //           ...A..B.
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t+s,1));   // A
  input.push_back(TimeAndTopic(t+s*3,0)); // b
  input.push_back(TimeAndTopic(t+s*4,1)); // B
  input.push_back(TimeAndTopic(t+s*6,0)); // c
  input.push_back(TimeAndTopic(t+s*7,1)); // C
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+s*3, t+s*4));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10);
  sync_test.run();
}


TEST(ApproxTimeSync, ImperfectMatch) {
  // Input A:  a.xb..c.
  // Input B:  .A...B.C
  // Output:   ..a...c.
  //           ..A...B.
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t+s,1));   // A
  input.push_back(TimeAndTopic(t+s*2,0)); // x
  input.push_back(TimeAndTopic(t+s*3,0)); // b
  input.push_back(TimeAndTopic(t+s*5,1)); // B
  input.push_back(TimeAndTopic(t+s*6,0)); // c
  input.push_back(TimeAndTopic(t+s*7,1)); // C
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+s*6, t+s*5));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10);
  sync_test.run();
}


TEST(ApproxTimeSync, Acceleration) {
  // Time:     0123456789012345678
  // Input A:  a...........b....c.
  // Input B:  .......A.......B..C
  // Output:   ............b.....c
  //           ............A.....C
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));      // a
  input.push_back(TimeAndTopic(t+s*7,1));  // A
  input.push_back(TimeAndTopic(t+s*12,0)); // b
  input.push_back(TimeAndTopic(t+s*15,1)); // B
  input.push_back(TimeAndTopic(t+s*17,0)); // c
  input.push_back(TimeAndTopic(t+s*18,1)); // C
  output.push_back(TimePair(t+s*12, t+s*7));
  output.push_back(TimePair(t+s*17, t+s*18));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10);
  sync_test.run();
}


TEST(ApproxTimeSync, DroppedMessages) {
  // Queue size 1 (too small)
  // Time:     012345678901234
  // Input A:  a...b...c.d..e.
  // Input B:  .A.B...C...D..E
  // Output:   .......b.....d.
  //           .......B.....D.
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t+s,1));   // A
  input.push_back(TimeAndTopic(t+s*3,1)); // B
  input.push_back(TimeAndTopic(t+s*4,0)); // b
  input.push_back(TimeAndTopic(t+s*7,1)); // C
  input.push_back(TimeAndTopic(t+s*8,0)); // c
  input.push_back(TimeAndTopic(t+s*10,0)); // d
  input.push_back(TimeAndTopic(t+s*11,1)); // D
  input.push_back(TimeAndTopic(t+s*13,0)); // e
  input.push_back(TimeAndTopic(t+s*14,1)); // E
  output.push_back(TimePair(t+s*4, t+s*3));
  output.push_back(TimePair(t+s*10, t+s*11));

  ApproximateTimeSynchronizerTest sync_test(input, output, 1);
  sync_test.run();

  // Queue size 2 (just enough)
  // Time:     012345678901234
  // Input A:  a...b...c.d..e.
  // Input B:  .A.B...C...D..E
  // Output:   ....a..b...c.d.
  //           ....A..B...C.D.
  std::vector<TimePair> output2;
  output2.push_back(TimePair(t, t+s));
  output2.push_back(TimePair(t+s*4, t+s*3));
  output2.push_back(TimePair(t+s*8, t+s*7));
  output2.push_back(TimePair(t+s*10, t+s*11));

  ApproximateTimeSynchronizerTest sync_test2(input, output2, 2);
  sync_test2.run();
}


TEST(ApproxTimeSync, LongQueue) {
  // Queue size 5
  // Time:     012345678901234
  // Input A:  abcdefghiklmnp.
  // Input B:  ...j......o....
  // Output:   ..........l....
  //           ..........o....
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t, 0));     // a
  input.push_back(TimeAndTopic(t + rclcpp::Duration(1, 0), 0));   // b
  input.push_back(TimeAndTopic(t + rclcpp::Duration(2, 0), 0));   // c
  input.push_back(TimeAndTopic(t + rclcpp::Duration(3, 0), 0));   // d
  input.push_back(TimeAndTopic(t + rclcpp::Duration(4, 0), 0));   // e
  input.push_back(TimeAndTopic(t + rclcpp::Duration(5, 0), 0));   // f
  input.push_back(TimeAndTopic(t + rclcpp::Duration(6, 0), 0));   // g
  input.push_back(TimeAndTopic(t + rclcpp::Duration(7, 0), 0));   // h
  input.push_back(TimeAndTopic(t + rclcpp::Duration(8, 0), 0));   // i
  input.push_back(TimeAndTopic(t + rclcpp::Duration(3, 0), 1));   // j
  input.push_back(TimeAndTopic(t + rclcpp::Duration(9, 0), 0));   // k
  input.push_back(TimeAndTopic(t + rclcpp::Duration(10, 0), 0));   // l
  input.push_back(TimeAndTopic(t + rclcpp::Duration(11, 0), 0));   // m
  input.push_back(TimeAndTopic(t + rclcpp::Duration(12, 0), 0));   // n
  input.push_back(TimeAndTopic(t + rclcpp::Duration(10, 0), 1));   // o
  input.push_back(TimeAndTopic(t + rclcpp::Duration(13, 0), 0));   // l
  output.push_back(TimePair(t + rclcpp::Duration(10, 0), t + rclcpp::Duration(10, 0)));

  ApproximateTimeSynchronizerTest sync_test(input, output, 5);
  sync_test.run();
}


TEST(ApproxTimeSync, DoublePublish) {
  // Input A:  a..b
  // Input B:  .A.B
  // Output:   ...b
  //           ...B
  //              +
  //              a
  //              A
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t+s,1));   // A
  input.push_back(TimeAndTopic(t+rclcpp::Duration(3, 0), 1)); // B
  input.push_back(TimeAndTopic(t+rclcpp::Duration(3, 0), 0)); // b
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+rclcpp::Duration(3, 0), t+rclcpp::Duration(3, 0)));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10);
  sync_test.run();
}


TEST(ApproxTimeSync, FourTopics) {
  // Time:     012345678901234
  // Input A:  a....e..i.m..n.
  // Input B:  .b....g..j....o
  // Input C:  ..c...h...k....
  // Input D:  ...d.f.....l...
  // Output:   ......a....e..m
  //           ......b....g..j
  //           ......c....h..k
  //           ......d....f..l
  std::vector<TimeAndTopic> input;
  std::vector<TimeQuad> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t+s,1));   // b
  input.push_back(TimeAndTopic(t+rclcpp::Duration(2, 0),2));   // c
  input.push_back(TimeAndTopic(t+rclcpp::Duration(3, 0),3));   // d 
  input.push_back(TimeAndTopic(t+rclcpp::Duration(5, 0),0));   // e
  input.push_back(TimeAndTopic(t+rclcpp::Duration(5, 0),3));   // f
  input.push_back(TimeAndTopic(t+rclcpp::Duration(6, 0),1));   // g
  input.push_back(TimeAndTopic(t+rclcpp::Duration(6, 0),2));   // h
  input.push_back(TimeAndTopic(t+rclcpp::Duration(8, 0),0));   // i
  input.push_back(TimeAndTopic(t+rclcpp::Duration(9, 0),1));   // j
  input.push_back(TimeAndTopic(t+rclcpp::Duration(10, 0),2));   // k
  input.push_back(TimeAndTopic(t+rclcpp::Duration(11, 0),3));   // l
  input.push_back(TimeAndTopic(t+rclcpp::Duration(10, 0),0));   // m
  input.push_back(TimeAndTopic(t+rclcpp::Duration(13, 0),0));   // n
  input.push_back(TimeAndTopic(t+rclcpp::Duration(14, 0),1));   // o
  output.push_back(TimeQuad(t, t+s, t+rclcpp::Duration(2, 0), t+rclcpp::Duration(3, 0)));
  output.push_back(TimeQuad(t+rclcpp::Duration(5, 0), t+rclcpp::Duration(6, 0), t+rclcpp::Duration(6, 0), t+rclcpp::Duration(5, 0)));
  output.push_back(TimeQuad(t+rclcpp::Duration(10, 0), t+rclcpp::Duration(9, 0), t+rclcpp::Duration(10, 0), t+rclcpp::Duration(11, 0)));

  ApproximateTimeSynchronizerTestQuad sync_test(input, output, 10);
  sync_test.run();
}


TEST(ApproxTimeSync, EarlyPublish) {
  // Time:     012345678901234
  // Input A:  a......e
  // Input B:  .b......
  // Input C:  ..c.....
  // Input D:  ...d....
  // Output:   .......a
  //           .......b
  //           .......c
  //           .......d
  std::vector<TimeAndTopic> input;
  std::vector<TimeQuad> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t+s,1));   // b
  input.push_back(TimeAndTopic(t+s*2,2));   // c
  input.push_back(TimeAndTopic(t+s*3,3));   // d 
  input.push_back(TimeAndTopic(t+s*7,0));   // e
  output.push_back(TimeQuad(t, t+s, t+s*2, t+s*3));

  ApproximateTimeSynchronizerTestQuad sync_test(input, output, 10);
  sync_test.run();
}


TEST(ApproxTimeSync, RateBound) {
  // Rate bound A: 1.5
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   .a..b...
  //           .A..B...
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t+s,1));   // A
  input.push_back(TimeAndTopic(t+s*3,0)); // b
  input.push_back(TimeAndTopic(t+s*4,1)); // B
  input.push_back(TimeAndTopic(t+s*6,0)); // c
  input.push_back(TimeAndTopic(t+s*7,1)); // C
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+s*3, t+s*4));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10);
  sync_test.sync_.setInterMessageLowerBound(0, s*1.5);
  sync_test.run();

  // Rate bound A: 2
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   .a..b..c
  //           .A..B..C

  output.push_back(TimePair(t+s*6, t+s*7));

  ApproximateTimeSynchronizerTest sync_test2(input, output, 10);
  sync_test2.sync_.setInterMessageLowerBound(0, s*2);
  sync_test2.run();

}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
