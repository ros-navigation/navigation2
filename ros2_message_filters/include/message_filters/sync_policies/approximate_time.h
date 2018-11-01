/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef MESSAGE_FILTERS_SYNC_APPROXIMATE_TIME_H
#define MESSAGE_FILTERS_SYNC_APPROXIMATE_TIME_H

#include "message_filters/synchronizer.h"
#include "message_filters/connection.h"
#include "message_filters/null_types.h"
#include "message_filters/signal9.h"
#include "message_filters/message_traits.h"

#include <rclcpp/rclcpp.hpp>
#include <cassert>
#include <deque>
#include <vector>
#include <string>
#include <tuple>

namespace message_filters
{
namespace sync_policies
{

template<typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
         typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
struct ApproximateTime : public PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
  typedef Synchronizer<ApproximateTime> Sync;
  typedef PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8> Super;
  typedef typename Super::Messages Messages;
  typedef typename Super::Signal Signal;
  typedef typename Super::Events Events;
  typedef typename Super::RealTypeCount RealTypeCount;
  typedef typename Super::M0Event M0Event;
  typedef typename Super::M1Event M1Event;
  typedef typename Super::M2Event M2Event;
  typedef typename Super::M3Event M3Event;
  typedef typename Super::M4Event M4Event;
  typedef typename Super::M5Event M5Event;
  typedef typename Super::M6Event M6Event;
  typedef typename Super::M7Event M7Event;
  typedef typename Super::M8Event M8Event;
  typedef std::deque<M0Event> M0Deque;
  typedef std::deque<M1Event> M1Deque;
  typedef std::deque<M2Event> M2Deque;
  typedef std::deque<M3Event> M3Deque;
  typedef std::deque<M4Event> M4Deque;
  typedef std::deque<M5Event> M5Deque;
  typedef std::deque<M6Event> M6Deque;
  typedef std::deque<M7Event> M7Deque;
  typedef std::deque<M8Event> M8Deque;
  typedef std::vector<M0Event> M0Vector;
  typedef std::vector<M1Event> M1Vector;
  typedef std::vector<M2Event> M2Vector;
  typedef std::vector<M3Event> M3Vector;
  typedef std::vector<M4Event> M4Vector;
  typedef std::vector<M5Event> M5Vector;
  typedef std::vector<M6Event> M6Vector;
  typedef std::vector<M7Event> M7Vector;
  typedef std::vector<M8Event> M8Vector;
//  typedef std::tuple<M0Event, M1Event, M2Event, M3Event, M4Event, M5Event, M6Event, M7Event, M8Event> Tuple;
  typedef Events Tuple;
  typedef std::tuple<M0Deque, M1Deque, M2Deque, M3Deque, M4Deque, M5Deque, M6Deque, M7Deque, M8Deque> DequeTuple;
  typedef std::tuple<M0Vector, M1Vector, M2Vector, M3Vector, M4Vector, M5Vector, M6Vector, M7Vector, M8Vector> VectorTuple;

  ApproximateTime(uint32_t queue_size)
  : parent_(0)
  , queue_size_(queue_size)
  , num_non_empty_deques_(0)
  , pivot_(NO_PIVOT)
  , max_interval_duration_(rclcpp::Duration(std::numeric_limits<int32_t>::max(),999999999))
  , age_penalty_(0.1)
  , has_dropped_messages_(9, false)
  , inter_message_lower_bounds_(9, rclcpp::Duration(0, 0))
  , warned_about_incorrect_bound_(9, false)
  {
    assert(queue_size_ > 0);  // The synchronizer will tend to drop many messages with a queue size of 1. At least 2 is recommended.
  }

  ApproximateTime(const ApproximateTime& e)
  : max_interval_duration_(rclcpp::Duration(std::numeric_limits<int32_t>::max(),999999999)) 
  {
    *this = e;
  }

  ApproximateTime& operator=(const ApproximateTime& rhs)
  {
    parent_ = rhs.parent_;
    queue_size_ = rhs.queue_size_;
    num_non_empty_deques_ = rhs.num_non_empty_deques_;
    pivot_time_ = rhs.pivot_time_;
    pivot_ = rhs.pivot_;
    max_interval_duration_ = rhs.max_interval_duration_;
    age_penalty_ = rhs.age_penalty_;
    candidate_start_ = rhs.candidate_start_;
    candidate_end_ = rhs.candidate_end_;
    deques_ = rhs.deques_;
    past_ = rhs.past_;
    has_dropped_messages_ = rhs.has_dropped_messages_;
    inter_message_lower_bounds_ = rhs.inter_message_lower_bounds_;
    warned_about_incorrect_bound_ = rhs.warned_about_incorrect_bound_;

    return *this;
  }

  void initParent(Sync* parent)
  {
    parent_ = parent;
  }

  template<int i>
  void checkInterMessageBound()
  {
    namespace mt = message_filters::message_traits;
    if (warned_about_incorrect_bound_[i])
    {
      return;
    }
    std::deque<typename std::tuple_element<i, Events>::type>& deque = std::get<i>(deques_);
    std::vector<typename std::tuple_element<i, Events>::type>& v = std::get<i>(past_);
    assert(!deque.empty());
    const typename std::tuple_element<i, Messages>::type &msg = *(deque.back()).getMessage();
    rclcpp::Time msg_time = mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(msg);
    rclcpp::Time previous_msg_time;
    if (deque.size() == (size_t) 1)
    {
      if (v.empty())
      {
	// We have already published (or have never received) the previous message, we cannot check the bound
	return;
      }
      const typename std::tuple_element<i, Messages>::type &previous_msg = *(v.back()).getMessage();
      previous_msg_time = mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(previous_msg);
    }
    else
    {
      // There are at least 2 elements in the deque. Check that the gap respects the bound if it was provided.
      const typename std::tuple_element<i, Messages>::type &previous_msg = *(deque[deque.size()-2]).getMessage();
      previous_msg_time = mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(previous_msg);
    }
    if (msg_time < previous_msg_time)
    {
      std::cout << "Messages of type " << i << " arrived out of order (will print only once)";
      warned_about_incorrect_bound_[i] = true;
    }
    else if ((msg_time - previous_msg_time) < inter_message_lower_bounds_[i])
    {
      std::cout << "Messages of type " << i << " arrived closer (" <<
        (msg_time - previous_msg_time).nanoseconds() <<
        ") than the lower bound you provided (" <<
        inter_message_lower_bounds_[i].nanoseconds() << ") (will print only once)";
      warned_about_incorrect_bound_[i] = true;
    }
  }


  template<int i>
  void add(const typename std::tuple_element<i, Events>::type& evt)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::deque<typename std::tuple_element<i, Events>::type>& deque = std::get<i>(deques_);
    deque.push_back(evt);
    if (deque.size() == (size_t)1) {
      // We have just added the first message, so it was empty before
      ++num_non_empty_deques_;
      if (num_non_empty_deques_ == (uint32_t)RealTypeCount::value)
      {
        // All deques have messages
        process();
      }
    }
    else
    {
      checkInterMessageBound<i>();
    }
    // Check whether we have more messages than allowed in the queue.
    // Note that during the above call to process(), queue i may contain queue_size_+1 messages.
    std::vector<typename std::tuple_element<i, Events>::type>& past = std::get<i>(past_);
    if (deque.size() + past.size() > queue_size_)
    {
      // Cancel ongoing candidate search, if any:
      num_non_empty_deques_ = 0; // We will recompute it from scratch
      recover<0>();
      recover<1>();
      recover<2>();
      recover<3>();
      recover<4>();
      recover<5>();
      recover<6>();
      recover<7>();
      recover<8>();
      // Drop the oldest message in the offending topic
      assert(!deque.empty());
      deque.pop_front();
      has_dropped_messages_[i] = true;
      if (pivot_ != NO_PIVOT)
      {
	// The candidate is no longer valid. Destroy it.
	candidate_ = Tuple();
	pivot_ = NO_PIVOT;
	// There might still be enough messages to create a new candidate:
	process();
      }
    }
  }

  void setAgePenalty(double age_penalty)
  {
    // For correctness we only need age_penalty > -1.0, but most likely a negative age_penalty is a mistake.
    assert(age_penalty >= 0);
    age_penalty_ = age_penalty;
  }

  void setInterMessageLowerBound(int i, rclcpp::Duration lower_bound) {
    // For correctness we only need age_penalty > -1.0, but most likely a negative age_penalty is a mistake.
    assert(lower_bound >= rclcpp::Duration(0,0));
    inter_message_lower_bounds_[i] = lower_bound;
  }

  void setMaxIntervalDuration(rclcpp::Duration max_interval_duration) {
    // For correctness we only need age_penalty > -1.0, but most likely a negative age_penalty is a mistake.
    assert(max_interval_duration >= rclcpp::Duration(0,0));
    max_interval_duration_ = max_interval_duration;
  }

private:
  // Assumes that deque number <index> is non empty
  template<int i>
  void dequeDeleteFront()
  {
    std::deque<typename std::tuple_element<i, Events>::type>& deque = std::get<i>(deques_);
    assert(!deque.empty());
    deque.pop_front();
    if (deque.empty())
    {
      --num_non_empty_deques_;
    }
  }

  // Assumes that deque number <index> is non empty
  void dequeDeleteFront(uint32_t index)
  {
    switch (index)
    {
    case 0:
      dequeDeleteFront<0>();
      break;
    case 1:
      dequeDeleteFront<1>();
      break;
    case 2:
      dequeDeleteFront<2>();
      break;
    case 3:
      dequeDeleteFront<3>();
      break;
    case 4:
      dequeDeleteFront<4>();
      break;
    case 5:
      dequeDeleteFront<5>();
      break;
    case 6:
      dequeDeleteFront<6>();
      break;
    case 7:
      dequeDeleteFront<7>();
      break;
    case 8:
      dequeDeleteFront<8>();
      break;
    default:
      std::abort();
    }
  }

  // Assumes that deque number <index> is non empty
  template<int i>
  void dequeMoveFrontToPast()
  {
    std::deque<typename std::tuple_element<i, Events>::type>& deque = std::get<i>(deques_);
    std::vector<typename std::tuple_element<i, Events>::type>& vector = std::get<i>(past_);
    assert(!deque.empty());
    vector.push_back(deque.front());
    deque.pop_front();
    if (deque.empty())
    {
      --num_non_empty_deques_;
    }
  }
  // Assumes that deque number <index> is non empty
  void dequeMoveFrontToPast(uint32_t index)
  {
    switch (index)
    {
    case 0:
      dequeMoveFrontToPast<0>();
      break;
    case 1:
      dequeMoveFrontToPast<1>();
      break;
    case 2:
      dequeMoveFrontToPast<2>();
      break;
    case 3:
      dequeMoveFrontToPast<3>();
      break;
    case 4:
      dequeMoveFrontToPast<4>();
      break;
    case 5:
      dequeMoveFrontToPast<5>();
      break;
    case 6:
      dequeMoveFrontToPast<6>();
      break;
    case 7:
      dequeMoveFrontToPast<7>();
      break;
    case 8:
      dequeMoveFrontToPast<8>();
      break;
    default:
      std::abort();
    }
  }

  void makeCandidate()
  {
    //printf("Creating candidate\n");
    // Create candidate tuple
    candidate_ = Tuple(); // Discards old one if any
    std::get<0>(candidate_) = std::get<0>(deques_).front();
    std::get<1>(candidate_) = std::get<1>(deques_).front();
    if (RealTypeCount::value > 2)
    {
      std::get<2>(candidate_) = std::get<2>(deques_).front();
      if (RealTypeCount::value > 3)
      {
	std::get<3>(candidate_) = std::get<3>(deques_).front();
	if (RealTypeCount::value > 4)
	{
	  std::get<4>(candidate_) = std::get<4>(deques_).front();
	  if (RealTypeCount::value > 5)
	  {
	    std::get<5>(candidate_) = std::get<5>(deques_).front();
	    if (RealTypeCount::value > 6)
	    {
	      std::get<6>(candidate_) = std::get<6>(deques_).front();
	      if (RealTypeCount::value > 7)
	      {
		std::get<7>(candidate_) = std::get<7>(deques_).front();
		if (RealTypeCount::value > 8)
		{
		  std::get<8>(candidate_) = std::get<8>(deques_).front();
		}
	      }
	    }
	  }
	}
      }
    }
    // Delete all past messages, since we have found a better candidate
    std::get<0>(past_).clear();
    std::get<1>(past_).clear();
    std::get<2>(past_).clear();
    std::get<3>(past_).clear();
    std::get<4>(past_).clear();
    std::get<5>(past_).clear();
    std::get<6>(past_).clear();
    std::get<7>(past_).clear();
    std::get<8>(past_).clear();
    //printf("Candidate created\n");
  }


  // ASSUMES: num_messages <= past_[i].size()
  template<int i>
  void recover(size_t num_messages)
  {
    if (i >= RealTypeCount::value)
    {
      return;
    }

    std::vector<typename std::tuple_element<i, Events>::type>& v = std::get<i>(past_);
    std::deque<typename std::tuple_element<i, Events>::type>& q = std::get<i>(deques_);
    assert(num_messages <= v.size());
    while (num_messages > 0)
    {
      q.push_front(v.back());
      v.pop_back();
      num_messages--;
    }

    if (!q.empty())
    {
      ++num_non_empty_deques_;
    }
  }


  template<int i>
  void recover()
  {
    if (i >= RealTypeCount::value)
    {
      return;
    }

    std::vector<typename std::tuple_element<i, Events>::type>& v = std::get<i>(past_);
    std::deque<typename std::tuple_element<i, Events>::type>& q = std::get<i>(deques_);
    while (!v.empty())
    {
      q.push_front(v.back());
      v.pop_back();
    }

    if (!q.empty())
    {
      ++num_non_empty_deques_;
    }
  }


  template<int i>
  void recoverAndDelete()
  {
    if (i >= RealTypeCount::value)
    {
      return;
    }

    std::vector<typename std::tuple_element<i, Events>::type>& v = std::get<i>(past_);
    std::deque<typename std::tuple_element<i, Events>::type>& q = std::get<i>(deques_);
    while (!v.empty())
    {
      q.push_front(v.back());
      v.pop_back();
    }

    assert(!q.empty());

    q.pop_front();
    if (!q.empty())
    {
      ++num_non_empty_deques_;
    }
  }

  // Assumes: all deques are non empty, i.e. num_non_empty_deques_ == RealTypeCount::value
  void publishCandidate()
  {
    //printf("Publishing candidate\n");
    // Publish
    parent_->signal(std::get<0>(candidate_), std::get<1>(candidate_), std::get<2>(candidate_), std::get<3>(candidate_),
                    std::get<4>(candidate_), std::get<5>(candidate_), std::get<6>(candidate_), std::get<7>(candidate_),
                    std::get<8>(candidate_));
    // Delete this candidate
    candidate_ = Tuple();
    pivot_ = NO_PIVOT;

    // Recover hidden messages, and delete the ones corresponding to the candidate
    num_non_empty_deques_ = 0; // We will recompute it from scratch
    recoverAndDelete<0>();
    recoverAndDelete<1>();
    recoverAndDelete<2>();
    recoverAndDelete<3>();
    recoverAndDelete<4>();
    recoverAndDelete<5>();
    recoverAndDelete<6>();
    recoverAndDelete<7>();
    recoverAndDelete<8>();
  }

  // Assumes: all deques are non empty, i.e. num_non_empty_deques_ == RealTypeCount::value
  // Returns: the oldest message on the deques
  void getCandidateStart(uint32_t &start_index, rclcpp::Time &start_time)
  {
    return getCandidateBoundary(start_index, start_time, false);
  }

  // Assumes: all deques are non empty, i.e. num_non_empty_deques_ == RealTypeCount::value
  // Returns: the latest message among the heads of the deques, i.e. the minimum
  //          time to end an interval started at getCandidateStart_index()
  void getCandidateEnd(uint32_t &end_index, rclcpp::Time &end_time)
  {
    return getCandidateBoundary(end_index, end_time, true);
  }

  // ASSUMES: all deques are non-empty
  // end = true: look for the latest head of deque
  //       false: look for the earliest head of deque
  void getCandidateBoundary(uint32_t &index, rclcpp::Time &time, bool end)
  {
    namespace mt = message_filters::message_traits;

    M0Event& m0 = std::get<0>(deques_).front();
    time = mt::TimeStamp<M0>::value(*m0.getMessage());
    index = 0;
    if (RealTypeCount::value > 1)
    {
      M1Event& m1 = std::get<1>(deques_).front();
      if ((mt::TimeStamp<M1>::value(*m1.getMessage()) < time) ^ end)
      {
        time = mt::TimeStamp<M1>::value(*m1.getMessage());
        index = 1;
      }
    }
    if (RealTypeCount::value > 2)
    {
      M2Event& m2 = std::get<2>(deques_).front();
      if ((mt::TimeStamp<M2>::value(*m2.getMessage()) < time) ^ end)
      {
        time = mt::TimeStamp<M2>::value(*m2.getMessage());
        index = 2;
      }
    }
    if (RealTypeCount::value > 3)
    {
      M3Event& m3 = std::get<3>(deques_).front();
      if ((mt::TimeStamp<M3>::value(*m3.getMessage()) < time) ^ end)
      {
        time = mt::TimeStamp<M3>::value(*m3.getMessage());
        index = 3;
      }
    }
    if (RealTypeCount::value > 4)
    {
      M4Event& m4 = std::get<4>(deques_).front();
      if ((mt::TimeStamp<M4>::value(*m4.getMessage()) < time) ^ end)
      {
        time = mt::TimeStamp<M4>::value(*m4.getMessage());
        index = 4;
      }
    }
    if (RealTypeCount::value > 5)
    {
      M5Event& m5 = std::get<5>(deques_).front();
      if ((mt::TimeStamp<M5>::value(*m5.getMessage()) < time) ^ end)
      {
        time = mt::TimeStamp<M5>::value(*m5.getMessage());
        index = 5;
      }
    }
    if (RealTypeCount::value > 6)
    {
      M6Event& m6 = std::get<6>(deques_).front();
      if ((mt::TimeStamp<M6>::value(*m6.getMessage()) < time) ^ end)
      {
        time = mt::TimeStamp<M6>::value(*m6.getMessage());
        index = 6;
      }
    }
    if (RealTypeCount::value > 7)
    {
      M7Event& m7 = std::get<7>(deques_).front();
      if ((mt::TimeStamp<M7>::value(*m7.getMessage()) < time) ^ end)
      {
        time = mt::TimeStamp<M7>::value(*m7.getMessage());
        index = 7;
      }
    }
    if (RealTypeCount::value > 8)
    {
      M8Event& m8 = std::get<8>(deques_).front();
      if ((mt::TimeStamp<M8>::value(*m8.getMessage()) < time) ^ end)
      {
        time = mt::TimeStamp<M8>::value(*m8.getMessage());
        index = 8;
      }
    }
  }


  // ASSUMES: we have a pivot and candidate
  template<int i>
  rclcpp::Time getVirtualTime()
  {
    namespace mt = message_filters::message_traits;

    if (i >= RealTypeCount::value)
    {
      return rclcpp::Time(0,0);  // Dummy return value
    }
    assert(pivot_ != NO_PIVOT);

    std::vector<typename std::tuple_element<i, Events>::type>& v = std::get<i>(past_);
    std::deque<typename std::tuple_element<i, Events>::type>& q = std::get<i>(deques_);
    if (q.empty())
    {
      assert(!v.empty());  // Because we have a candidate
      rclcpp::Time last_msg_time = mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(*(v.back()).getMessage());
      rclcpp::Time msg_time_lower_bound = last_msg_time + inter_message_lower_bounds_[i];
      if (msg_time_lower_bound > pivot_time_)  // Take the max
      {
        return msg_time_lower_bound;
      }
      return pivot_time_;
    }
    rclcpp::Time current_msg_time = mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(*(q.front()).getMessage());
    return current_msg_time;
  }


  // ASSUMES: we have a pivot and candidate
  void getVirtualCandidateStart(uint32_t &start_index, rclcpp::Time &start_time)
  {
    return getVirtualCandidateBoundary(start_index, start_time, false);
  }

  // ASSUMES: we have a pivot and candidate
  void getVirtualCandidateEnd(uint32_t &end_index, rclcpp::Time &end_time)
  {
    return getVirtualCandidateBoundary(end_index, end_time, true);
  }

  // ASSUMES: we have a pivot and candidate
  // end = true: look for the latest head of deque
  //       false: look for the earliest head of deque
  void getVirtualCandidateBoundary(uint32_t &index, rclcpp::Time &time, bool end)
  {
    std::vector<rclcpp::Time> virtual_times(9);
    virtual_times[0] = getVirtualTime<0>();
    virtual_times[1] = getVirtualTime<1>();
    virtual_times[2] = getVirtualTime<2>();
    virtual_times[3] = getVirtualTime<3>();
    virtual_times[4] = getVirtualTime<4>();
    virtual_times[5] = getVirtualTime<5>();
    virtual_times[6] = getVirtualTime<6>();
    virtual_times[7] = getVirtualTime<7>();
    virtual_times[8] = getVirtualTime<8>();
 
    time = virtual_times[0];
    index = 0;
    for (int i = 0; i < RealTypeCount::value; i++)
    {
      if ((virtual_times[i] < time) ^ end)
      {
	time = virtual_times[i];
	index = i;
      }
    }
  }


  // assumes data_mutex_ is already locked
  void process()
  {
    // While no deque is empty
    while (num_non_empty_deques_ == (uint32_t)RealTypeCount::value)
    {
      // Find the start and end of the current interval
      //printf("Entering while loop in this state [\n");
      //show_internal_state();
      //printf("]\n");
      rclcpp::Time end_time, start_time;
      uint32_t end_index, start_index;
      getCandidateEnd(end_index, end_time);
      getCandidateStart(start_index, start_time);
      for (uint32_t i = 0; i < (uint32_t)RealTypeCount::value; i++)
      {
	if (i != end_index)
	{
	  // No dropped message could have been better to use than the ones we have,
	  // so it becomes ok to use this topic as pivot in the future
	  has_dropped_messages_[i] = false;
	}
      }
      if (pivot_ == NO_PIVOT)
      {
        // We do not have a candidate
        // INVARIANT: the past_ vectors are empty
        // INVARIANT: (candidate_ has no filled members)
        if (end_time - start_time > max_interval_duration_)
        {
          // This interval is too big to be a valid candidate, move to the next
          dequeDeleteFront(start_index);
          continue;
        }
	if (has_dropped_messages_[end_index])
	{
	  // The topic that would become pivot has dropped messages, so it is not a good pivot
	  dequeDeleteFront(start_index);
	  continue;
	}
	// This is a valid candidate, and we don't have any, so take it
	makeCandidate();
	candidate_start_ = start_time;
	candidate_end_ = end_time;
	pivot_ = end_index;
	pivot_time_ = end_time;
	dequeMoveFrontToPast(start_index);
      }
      else
      {
        // We already have a candidate
        // Is this one better than the current candidate?
        // INVARIANT: has_dropped_messages_ is all false
        if ((end_time - candidate_end_) * (1 + age_penalty_) >= (start_time - candidate_start_))
        {
          // This is not a better candidate, move to the next
          dequeMoveFrontToPast(start_index);
        }
        else
        {
          // This is a better candidate
          makeCandidate();
          candidate_start_ = start_time;
          candidate_end_ = end_time;
          dequeMoveFrontToPast(start_index);
          // Keep the same pivot (and pivot time)
        }
      }
      // INVARIANT: we have a candidate and pivot
      assert(pivot_ != NO_PIVOT);
      //printf("start_index == %d, pivot_ == %d\n", start_index, pivot_);
      if (start_index == pivot_)  // TODO: replace with start_time == pivot_time_
      {
        // We have exhausted all possible candidates for this pivot, we now can output the best one
        publishCandidate();
      }
      else if ((end_time - candidate_end_) * (1 + age_penalty_) >= (pivot_time_ - candidate_start_))
      {
        // We have not exhausted all candidates, but this candidate is already provably optimal
        // Indeed, any future candidate must contain the interval [pivot_time_ end_time], which
        // is already too big.
        // Note: this case is subsumed by the next, but it may save some unnecessary work and
        //       it makes things (a little) easier to understand
        publishCandidate();
      }
      else if (num_non_empty_deques_ < (uint32_t)RealTypeCount::value)
      {
        uint32_t num_non_empty_deques_before_virtual_search = num_non_empty_deques_;

        // Before giving up, use the rate bounds, if provided, to further try to prove optimality
        std::vector<int> num_virtual_moves(9,0);
        while (1)
        {
          rclcpp::Time end_time, start_time;
          uint32_t end_index, start_index;
          getVirtualCandidateEnd(end_index, end_time);
          getVirtualCandidateStart(start_index, start_time);
          if ((end_time - candidate_end_) * (1 + age_penalty_) >= (pivot_time_ - candidate_start_))
          {
            // We have proved optimality
            // As above, any future candidate must contain the interval [pivot_time_ end_time], which
            // is already too big.
            publishCandidate();  // This cleans up the virtual moves as a byproduct
            break;  // From the while(1) loop only
          }
          if ((end_time - candidate_end_) * (1 + age_penalty_) < (start_time - candidate_start_))
          {
            // We cannot prove optimality
            // Indeed, we have a virtual (i.e. optimistic) candidate that is better than the current
            // candidate
            // Cleanup the virtual search:
            num_non_empty_deques_ = 0; // We will recompute it from scratch
	    recover<0>(num_virtual_moves[0]);
	    recover<1>(num_virtual_moves[1]);
	    recover<2>(num_virtual_moves[2]);
	    recover<3>(num_virtual_moves[3]);
	    recover<4>(num_virtual_moves[4]);
	    recover<5>(num_virtual_moves[5]);
	    recover<6>(num_virtual_moves[6]);
	    recover<7>(num_virtual_moves[7]);
	    recover<8>(num_virtual_moves[8]);
            (void)num_non_empty_deques_before_virtual_search; // unused variable warning stopper
            assert(num_non_empty_deques_before_virtual_search == num_non_empty_deques_);
            break;
          }
          // Note: we cannot reach this point with start_index == pivot_ since in that case we would
          //       have start_time == pivot_time, in which case the two tests above are the negation
          //       of each other, so that one must be true. Therefore the while loop always terminates.
	  assert(start_index != pivot_);
	  assert(start_time < pivot_time_);
          dequeMoveFrontToPast(start_index);
          num_virtual_moves[start_index]++;
        } // while(1)
      }
    } // while(num_non_empty_deques_ == (uint32_t)RealTypeCount::value)
  }

  Sync* parent_;
  uint32_t queue_size_;

  static const uint32_t NO_PIVOT = 9;  // Special value for the pivot indicating that no pivot has been selected

  DequeTuple deques_;
  uint32_t num_non_empty_deques_;
  VectorTuple past_;
  Tuple candidate_;  // NULL if there is no candidate, in which case there is no pivot.
  rclcpp::Time candidate_start_;
  rclcpp::Time candidate_end_;
  rclcpp::Time pivot_time_;
  uint32_t pivot_;  // Equal to NO_PIVOT if there is no candidate
  std::mutex data_mutex_;  // Protects all of the above

  rclcpp::Duration max_interval_duration_; // TODO: initialize with a parameter
  double age_penalty_;

  std::vector<bool> has_dropped_messages_;
  std::vector<rclcpp::Duration> inter_message_lower_bounds_;
  std::vector<bool> warned_about_incorrect_bound_;
};

} // namespace sync
} // namespace message_filters

#endif // MESSAGE_FILTERS_SYNC_APPROXIMATE_TIME_H

