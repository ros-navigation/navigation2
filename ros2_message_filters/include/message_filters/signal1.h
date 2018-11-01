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

#ifndef MESSAGE_FILTERS_SIGNAL1_H
#define MESSAGE_FILTERS_SIGNAL1_H

#include <memory>
#include "connection.h"
#include "message_event.h"
#include "parameter_adapter.h"

#include <mutex>

namespace message_filters
{
template<class M>
class CallbackHelper1
{
public:
  virtual ~CallbackHelper1() {}

  virtual void call(const MessageEvent<M const>& event, bool nonconst_need_copy) = 0;

  typedef std::shared_ptr<CallbackHelper1<M> > Ptr;
};

template<typename P, typename M>
class CallbackHelper1T : public CallbackHelper1<M>
{
public:
  typedef ParameterAdapter<P> Adapter;
  typedef std::function<void(typename Adapter::Parameter)> Callback;
  typedef typename Adapter::Event Event;

  CallbackHelper1T(const Callback& cb)
  : callback_(cb)
  {
  }

  virtual void call(const MessageEvent<M const>& event, bool nonconst_force_copy)
  {
    Event my_event(event, nonconst_force_copy || event.nonConstWillCopy());
    callback_(Adapter::getParameter(my_event));
  }

private:
  Callback callback_;
};

template<class M>
class Signal1
{
  typedef std::shared_ptr<CallbackHelper1<M> > CallbackHelper1Ptr;
  typedef std::vector<CallbackHelper1Ptr> V_CallbackHelper1;

public:
  template<typename P>
  CallbackHelper1Ptr addCallback(const std::function<void(P)>& callback)
  {
    CallbackHelper1T<P, M>* helper = new CallbackHelper1T<P, M>(callback);

    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_.push_back(CallbackHelper1Ptr(helper));
    return callbacks_.back();
  }

  void removeCallback(const CallbackHelper1Ptr& helper)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    typename V_CallbackHelper1::iterator it = std::find(callbacks_.begin(), callbacks_.end(), helper);
    if (it != callbacks_.end())
    {
      callbacks_.erase(it);
    }
  }

  void call(const MessageEvent<M const>& event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    bool nonconst_force_copy = callbacks_.size() > 1;
    typename V_CallbackHelper1::iterator it = callbacks_.begin();
    typename V_CallbackHelper1::iterator end = callbacks_.end();
    for (; it != end; ++it)
    {
      const CallbackHelper1Ptr& helper = *it;
      helper->call(event, nonconst_force_copy);
    }
  }

private:
  std::mutex mutex_;
  V_CallbackHelper1 callbacks_;
};

} // message_filters

#endif // MESSAGE_FILTERS_SIGNAL1_H


