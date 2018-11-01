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

#ifndef MESSAGE_FILTERS_PASSTHROUGH_H
#define MESSAGE_FILTERS_PASSTHROUGH_H

#include "simple_filter.h"

#include <vector>

namespace message_filters
{
/**
 * \brief Simple passthrough filter.  What comes in goes out immediately.
 */
template<typename M>
class PassThrough : public SimpleFilter<M>
{
public:
  typedef std::shared_ptr<M const> MConstPtr;
  typedef MessageEvent<M const> EventType;

  PassThrough()
  {
  }

  template<typename F>
  PassThrough(F& f)
  {
    connectInput(f);
  }

  template<class F>
  void connectInput(F& f)
  {
    incoming_connection_.disconnect();
    incoming_connection_ = f.registerCallback(typename SimpleFilter<M>::EventCallback(std::bind(&PassThrough::cb, this, _1)));
  }

  void add(const MConstPtr& msg)
  {
    add(EventType(msg));
  }

  void add(const EventType& evt)
  {
    this->signalMessage(evt);
  }

private:
  void cb(const EventType& evt)
  {
    add(evt);
  }

  Connection incoming_connection_;
};

} // namespace message_filters

#endif // MESSAGE_FILTERS_PASSTHROUGH_H

