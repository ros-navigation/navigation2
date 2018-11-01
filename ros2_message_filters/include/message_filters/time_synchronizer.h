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

#ifndef MESSAGE_FILTERS_TIME_SYNCHRONIZER_H
#define MESSAGE_FILTERS_TIME_SYNCHRONIZER_H

#include "synchronizer.h"
#include "sync_policies/exact_time.h"

#include <memory>
#include "message_event.h"

namespace message_filters
{

/**
 * \brief Synchronizes up to 9 messages by their timestamps.
 *
 * TimeSynchronizer synchronizes up to 9 incoming channels by the timestamps contained in their messages' headers.
 * TimeSynchronizer takes anywhere from 2 to 9 message types as template parameters, and passes them through to a
 * callback which takes a shared pointer of each.
 *
 * The required queue size parameter when constructing the TimeSynchronizer tells it how many sets of messages it should
 * store (by timestamp) while waiting for messages to arrive and complete their "set"
 *
 * \section connections CONNECTIONS
 *
 * The input connections for the TimeSynchronizer object is the same signature as for roscpp subscription callbacks, ie.
\verbatim
void callback(const std::shared_ptr<M const>&);
\endverbatim
 * The output connection for the TimeSynchronizer object is dependent on the number of messages being synchronized.  For
 * a 3-message synchronizer for example, it would be:
\verbatim
void callback(const std::shared_ptr<M0 const>&, const std::shared_ptr<M1 const>&, const std::shared_ptr<M2 const>&);
\endverbatim
 * \section usage USAGE
 * Example usage would be:
\verbatim
TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::Image> sync_policies(caminfo_sub, limage_sub, rimage_sub, 3);
sync_policies.registerCallback(callback);
\endverbatim

 * The callback is then of the form:
\verbatim
void callback(const sensor_msgs::CameraInfo::ConstPtr&, const sensor_msgs::Image::ConstPtr&, const sensor_msgs::Image::ConstPtr&);
\endverbatim
 *
 */
template<class M0, class M1, class M2 = NullType, class M3 = NullType, class M4 = NullType,
         class M5 = NullType, class M6 = NullType, class M7 = NullType, class M8 = NullType>
class TimeSynchronizer : public Synchronizer<sync_policies::ExactTime<M0, M1, M2, M3, M4, M5, M6, M7, M8> >
{
public:
  typedef sync_policies::ExactTime<M0, M1, M2, M3, M4, M5, M6, M7, M8> Policy;
  typedef Synchronizer<Policy> Base;
  typedef std::shared_ptr<M0 const> M0ConstPtr;
  typedef std::shared_ptr<M1 const> M1ConstPtr;
  typedef std::shared_ptr<M2 const> M2ConstPtr;
  typedef std::shared_ptr<M3 const> M3ConstPtr;
  typedef std::shared_ptr<M4 const> M4ConstPtr;
  typedef std::shared_ptr<M5 const> M5ConstPtr;
  typedef std::shared_ptr<M6 const> M6ConstPtr;
  typedef std::shared_ptr<M7 const> M7ConstPtr;
  typedef std::shared_ptr<M8 const> M8ConstPtr;

  using Base::add;
  using Base::connectInput;
  using Base::registerCallback;
  using Base::setName;
  using Base::getName;
  using Policy::registerDropCallback;
  typedef typename Base::M0Event M0Event;
  typedef typename Base::M1Event M1Event;
  typedef typename Base::M2Event M2Event;
  typedef typename Base::M3Event M3Event;
  typedef typename Base::M4Event M4Event;
  typedef typename Base::M5Event M5Event;
  typedef typename Base::M6Event M6Event;
  typedef typename Base::M7Event M7Event;
  typedef typename Base::M8Event M8Event;

  template<class F0, class F1>
  TimeSynchronizer(F0& f0, F1& f1, uint32_t queue_size)
  : Base(Policy(queue_size))
  {
    connectInput(f0, f1);
  }

  template<class F0, class F1, class F2>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, uint32_t queue_size)
  : Base(Policy(queue_size))
  {
    connectInput(f0, f1, f2);
  }

  template<class F0, class F1, class F2, class F3>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, uint32_t queue_size)
  : Base(Policy(queue_size))
  {
    connectInput(f0, f1, f2, f3);
  }

  template<class F0, class F1, class F2, class F3, class F4>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, uint32_t queue_size)
  : Base(Policy(queue_size))
  {
    connectInput(f0, f1, f2, f3, f4);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, uint32_t queue_size)
  : Base(Policy(queue_size))
  {
    connectInput(f0, f1, f2, f3, f4, f5);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, uint32_t queue_size)
  : Base(Policy(queue_size))
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7, uint32_t queue_size)
  : Base(Policy(queue_size))
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7, class F8>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7, F8& f8, uint32_t queue_size)
  : Base(Policy(queue_size))
  {
    connectInput(f0, f1, f2, f3, f4, f5, f6, f7, f8);
  }

  TimeSynchronizer(uint32_t queue_size)
  : Base(Policy(queue_size))
  {
  }

  ////////////////////////////////////////////////////////////////
  // For backwards compatibility
  ////////////////////////////////////////////////////////////////
  void add0(const M0ConstPtr& msg)
  {
    this->template add<0>(M0Event(msg));
  }

  void add1(const M1ConstPtr& msg)
  {
    this->template add<1>(M1Event(msg));
  }

  void add2(const M2ConstPtr& msg)
  {
    this->template add<2>(M2Event(msg));
  }

  void add3(const M3ConstPtr& msg)
  {
    this->template add<3>(M3Event(msg));
  }

  void add4(const M4ConstPtr& msg)
  {
    this->template add<4>(M4Event(msg));
  }

  void add5(const M5ConstPtr& msg)
  {
    this->template add<5>(M5Event(msg));
  }

  void add6(const M6ConstPtr& msg)
  {
    this->template add<6>(M6Event(msg));
  }

  void add7(const M7ConstPtr& msg)
  {
    this->template add<7>(M7Event(msg));
  }

  void add8(const M8ConstPtr& msg)
  {
    this->template add<8>(M8Event(msg));
  }
};

}

#endif // MESSAGE_FILTERS_TIME_SYNCHRONIZER_H
