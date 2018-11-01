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

#ifndef MESSAGE_FILTERS_CACHE_H_
#define MESSAGE_FILTERS_CACHE_H_

#include <deque>
#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "connection.h"
#include "simple_filter.h"
#include "message_traits.h"

namespace message_filters
{
using namespace std::placeholders;
/**
 * \brief Stores a time history of messages
 *
 * Given a stream of messages, the most recent N messages are cached in a ring buffer,
 * from which time intervals of the cache can then be retrieved by the client.
 *
 * Cache immediately passes messages through to its output connections.
 *
 * \section connections CONNECTIONS
 *
 * Cache's input and output connections are both of the same signature as roscpp subscription callbacks, ie.
\verbatim
void callback(const std::shared_ptr<M const>&);
\endverbatim
 */
template<class M>
class Cache : public SimpleFilter<M>
{
public:
  typedef std::shared_ptr<M const> MConstPtr;
  typedef MessageEvent<M const> EventType;

  template<class F>
  Cache(F& f, unsigned int cache_size = 1)
  {
    setCacheSize(cache_size) ;
    connectInput(f) ;
  }

  /**
   * Initializes a Message Cache without specifying a parent filter. This implies that in
   * order to populate the cache, the user then has to call add themselves, or connectInput() is
   * called later
   */
  Cache(unsigned int cache_size = 1)
  {
    setCacheSize(cache_size);
  }

  template<class F>
  void connectInput(F& f)
  {
    incoming_connection_ = f.registerCallback(typename SimpleFilter<M>::EventCallback(std::bind(&Cache::callback, this, _1)));
  }

  ~Cache()
  {
    incoming_connection_.disconnect();
  }

  /**
   * Set the size of the cache.
   * \param cache_size The new size the cache should be. Must be > 0
   */
  void setCacheSize(unsigned int cache_size)
  {
    if (cache_size == 0)
    {
      //ROS_ERROR("Cannot set max_size to 0") ;
      return ;
    }

    cache_size_ = cache_size ;
  }

  /**
   * \brief Add a message to the cache, and pop off any elements that are too old.
   * This method is registered with a data provider when connectTo is called.
   */
  void add(const MConstPtr& msg)
  {
    add(EventType(msg));
  }

  /**
   * \brief Add a message to the cache, and pop off any elements that are too old.
   * This method is registered with a data provider when connectTo is called.
   */
  void add(const EventType& evt)
  {
//    namespace mt = ros::message_traits;
    namespace mt = message_filters::message_traits;

    //printf("  Cache Size: %u\n", cache_.size()) ;
    {
      std::lock_guard<std::mutex> lock(cache_lock_);

      while (cache_.size() >= cache_size_)                       // Keep popping off old data until we have space for a new msg
        cache_.pop_front() ;                                     // The front of the deque has the oldest elem, so we can get rid of it

      // No longer naively pushing msgs to back. Want to make sure they're sorted correctly
      //cache_.push_back(msg) ;                                    // Add the newest message to the back of the deque

      typename std::deque<EventType >::reverse_iterator rev_it = cache_.rbegin();

      // Keep walking backwards along deque until we hit the beginning,
      //   or until we find a timestamp that's smaller than (or equal to) msg's timestamp
      rclcpp::Time evt_stamp = mt::TimeStamp<M>::value(*evt.getMessage());
      while(rev_it != cache_.rend() && mt::TimeStamp<M>::value(*(*rev_it).getMessage()) > evt_stamp)
        rev_it++;

      // Add msg to the cache
      cache_.insert(rev_it.base(), evt);

    }

    this->signalMessage(evt);
  }

  /**
   * \brief Receive a vector of messages that occur between a start and end time (inclusive).
   *
   * This call is non-blocking, and only aggregates messages it has already received.
   * It will not wait for messages have not yet been received, but occur in the interval.
   * \param start The start of the requested interval
   * \param end The end of the requested interval
   */
  std::vector<MConstPtr> getInterval(const rclcpp::Time& start, const rclcpp::Time& end) const
  {
    namespace mt = message_filters::message_traits;
    std::lock_guard<std::mutex> lock(cache_lock_);

    // Find the starting index. (Find the first index after [or at] the start of the interval)
    unsigned int start_index = 0 ;
    while(start_index < cache_.size() &&
          mt::TimeStamp<M>::value(*cache_[start_index].getMessage()) < start)
    {
      start_index++ ;
    }

    // Find the ending index. (Find the first index after the end of interval)
    unsigned int end_index = start_index ;
    while(end_index < cache_.size() &&
          mt::TimeStamp<M>::value(*cache_[end_index].getMessage()) <= end)
    {
      end_index++ ;
    }

    std::vector<MConstPtr> interval_elems ;
    interval_elems.reserve(end_index - start_index) ;
    for (unsigned int i=start_index; i<end_index; i++)
    {
      interval_elems.push_back(cache_[i].getMessage()) ;
    }

    return interval_elems ;
  }


  /**
   * \brief Retrieve the smallest interval of messages that surrounds an interval from start to end.
   *
   * If the messages in the cache do not surround (start,end), then this will return the interval
   * that gets closest to surrounding (start,end)
   */
  std::vector<MConstPtr> getSurroundingInterval(const rclcpp::Time& start, const rclcpp::Time& end) const
  {
    namespace mt = message_filters::message_traits;

    std::lock_guard<std::mutex> lock(cache_lock_);
    // Find the starting index. (Find the first index after [or at] the start of the interval)
    unsigned int start_index = cache_.size()-1;
    while(start_index > 0 &&
          mt::TimeStamp<M>::value(*cache_[start_index].getMessage()) > start)
    {
      start_index--;
    }
    unsigned int end_index = start_index;
    while(end_index < cache_.size()-1 &&
          mt::TimeStamp<M>::value(*cache_[end_index].getMessage()) < end)
    {
      end_index++;
    }

    std::vector<MConstPtr> interval_elems;
    interval_elems.reserve(end_index - start_index + 1) ;
    for (unsigned int i=start_index; i<=end_index; i++)
    {
      interval_elems.push_back(cache_[i].getMessage()) ;
    }

    return interval_elems;
  }

  /**
   * \brief Grab the newest element that occurs right before the specified time.
   * \param time Time that must occur right after the returned elem
   * \returns shared_ptr to the newest elem that occurs before 'time'. NULL if doesn't exist
   */
  MConstPtr getElemBeforeTime(const rclcpp::Time& time) const
  {
    namespace mt = message_filters::message_traits;

    std::lock_guard<std::mutex> lock(cache_lock_);

    MConstPtr out ;

    unsigned int i=0 ;
    int elem_index = -1 ;
    while (i<cache_.size() &&
           mt::TimeStamp<M>::value(*cache_[i].getMessage()) < time)
    {
      elem_index = i ;
      i++ ;
    }

    if (elem_index >= 0)
      out = cache_[elem_index].getMessage() ;

    return out ;
  }

  /**
   * \brief Grab the oldest element that occurs right after the specified time.
   * \param time Time that must occur right before the returned elem
   * \returns shared_ptr to the oldest elem that occurs after 'time'. NULL if doesn't exist
   */
  MConstPtr getElemAfterTime(const rclcpp::Time& time) const
  {
    namespace mt = message_filters::message_traits;

    std::lock_guard<std::mutex> lock(cache_lock_);

    MConstPtr out ;

    int i=cache_.size()-1 ;
    int elem_index = -1 ;
    while (i>=0 &&
        mt::TimeStamp<M>::value(*cache_[i].getMessage()) > time)
    {
      elem_index = i ;
      i-- ;
    }

    if (elem_index >= 0)
      out = cache_[elem_index].getMessage() ;
    else
      out.reset() ;

    return out ;
  }

  /**
   * \brief Returns the timestamp associated with the newest packet cache
   */
  rclcpp::Time getLatestTime() const
  {
    namespace mt = message_filters::message_traits;

    std::lock_guard<std::mutex> lock(cache_lock_);

    rclcpp::Time latest_time;

    if (cache_.size() > 0)
      latest_time = mt::TimeStamp<M>::value(*cache_.back().getMessage());

    return latest_time ;
  }

  /**
   * \brief Returns the timestamp associated with the oldest packet cache
   */
  rclcpp::Time getOldestTime() const
  {
    namespace mt = message_filters::message_traits;

    std::lock_guard<std::mutex> lock(cache_lock_);

    rclcpp::Time oldest_time;

    if (cache_.size() > 0)
      oldest_time = mt::TimeStamp<M>::value(*cache_.front().getMessage());

    return oldest_time ;
  }


private:
  void callback(const EventType& evt)
  {
    add(evt);
  }

  mutable std::mutex cache_lock_ ;            //!< Lock for cache_
  std::deque<EventType> cache_ ;        //!< Cache for the messages
  unsigned int cache_size_ ;            //!< Maximum number of elements allowed in the cache.

  Connection incoming_connection_;
};

}


#endif /* MESSAGE_FILTERS_CACHE_H_ */
