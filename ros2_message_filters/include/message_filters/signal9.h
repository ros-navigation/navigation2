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

#ifndef MESSAGE_FILTERS_SIGNAL9_H
#define MESSAGE_FILTERS_SIGNAL9_H


#include <mutex>
#include "connection.h"
#include "null_types.h"
#include "message_event.h"
#include "parameter_adapter.h"

namespace message_filters
{
using namespace std::placeholders;
template<typename M0, typename M1, typename M2, typename M3, typename M4, typename M5, typename M6, typename M7, typename M8>
class CallbackHelper9
{
public:
  typedef MessageEvent<M0 const> M0Event;
  typedef MessageEvent<M1 const> M1Event;
  typedef MessageEvent<M2 const> M2Event;
  typedef MessageEvent<M3 const> M3Event;
  typedef MessageEvent<M4 const> M4Event;
  typedef MessageEvent<M5 const> M5Event;
  typedef MessageEvent<M6 const> M6Event;
  typedef MessageEvent<M7 const> M7Event;
  typedef MessageEvent<M8 const> M8Event;

  virtual ~CallbackHelper9() {}

  virtual void call(bool nonconst_force_copy, const M0Event& e0, const M1Event& e1, const M2Event& e2, const M3Event& e3,
                    const M4Event& e4, const M5Event& e5, const M6Event& e6, const M7Event& e7, const M8Event& e8) = 0;

  typedef std::shared_ptr<CallbackHelper9> Ptr;
};

template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
class CallbackHelper9T :
  public CallbackHelper9<typename ParameterAdapter<P0>::Message,
                         typename ParameterAdapter<P1>::Message,
                         typename ParameterAdapter<P2>::Message,
                         typename ParameterAdapter<P3>::Message,
                         typename ParameterAdapter<P4>::Message,
                         typename ParameterAdapter<P5>::Message,
                         typename ParameterAdapter<P6>::Message,
                         typename ParameterAdapter<P7>::Message,
                         typename ParameterAdapter<P8>::Message>
{
private:
  typedef ParameterAdapter<P0> A0;
  typedef ParameterAdapter<P1> A1;
  typedef ParameterAdapter<P2> A2;
  typedef ParameterAdapter<P3> A3;
  typedef ParameterAdapter<P4> A4;
  typedef ParameterAdapter<P5> A5;
  typedef ParameterAdapter<P6> A6;
  typedef ParameterAdapter<P7> A7;
  typedef ParameterAdapter<P8> A8;
  typedef typename A0::Event M0Event;
  typedef typename A1::Event M1Event;
  typedef typename A2::Event M2Event;
  typedef typename A3::Event M3Event;
  typedef typename A4::Event M4Event;
  typedef typename A5::Event M5Event;
  typedef typename A6::Event M6Event;
  typedef typename A7::Event M7Event;
  typedef typename A8::Event M8Event;

public:
  typedef std::function<void(typename A0::Parameter, typename A1::Parameter, typename A2::Parameter,
                               typename A3::Parameter, typename A4::Parameter, typename A5::Parameter,
                               typename A6::Parameter, typename A7::Parameter, typename A8::Parameter)> Callback;

  CallbackHelper9T(const Callback& cb)
  : callback_(cb)
  {
  }

  virtual void call(bool nonconst_force_copy, const M0Event& e0, const M1Event& e1, const M2Event& e2, const M3Event& e3,
                    const M4Event& e4, const M5Event& e5, const M6Event& e6, const M7Event& e7, const M8Event& e8)
  {
    M0Event my_e0(e0, nonconst_force_copy || e0.nonConstWillCopy());
    M1Event my_e1(e1, nonconst_force_copy || e0.nonConstWillCopy());
    M2Event my_e2(e2, nonconst_force_copy || e0.nonConstWillCopy());
    M3Event my_e3(e3, nonconst_force_copy || e0.nonConstWillCopy());
    M4Event my_e4(e4, nonconst_force_copy || e0.nonConstWillCopy());
    M5Event my_e5(e5, nonconst_force_copy || e0.nonConstWillCopy());
    M6Event my_e6(e6, nonconst_force_copy || e0.nonConstWillCopy());
    M7Event my_e7(e7, nonconst_force_copy || e0.nonConstWillCopy());
    M8Event my_e8(e8, nonconst_force_copy || e0.nonConstWillCopy());
    callback_(A0::getParameter(e0),
              A1::getParameter(e1),
              A2::getParameter(e2),
              A3::getParameter(e3),
              A4::getParameter(e4),
              A5::getParameter(e5),
              A6::getParameter(e6),
              A7::getParameter(e7),
              A8::getParameter(e8));
  }

private:
  Callback callback_;
};

template<typename M0, typename M1, typename M2, typename M3, typename M4, typename M5, typename M6, typename M7, typename M8>
class Signal9
{
  typedef std::shared_ptr<CallbackHelper9<M0, M1, M2, M3, M4, M5, M6, M7, M8> > CallbackHelper9Ptr;
  typedef std::vector<CallbackHelper9Ptr> V_CallbackHelper9;

public:
  typedef MessageEvent<M0 const> M0Event;
  typedef MessageEvent<M1 const> M1Event;
  typedef MessageEvent<M2 const> M2Event;
  typedef MessageEvent<M3 const> M3Event;
  typedef MessageEvent<M4 const> M4Event;
  typedef MessageEvent<M5 const> M5Event;
  typedef MessageEvent<M6 const> M6Event;
  typedef MessageEvent<M7 const> M7Event;
  typedef MessageEvent<M8 const> M8Event;
  typedef std::shared_ptr<M0 const> M0ConstPtr;
  typedef std::shared_ptr<M1 const> M1ConstPtr;
  typedef std::shared_ptr<M2 const> M2ConstPtr;
  typedef std::shared_ptr<M3 const> M3ConstPtr;
  typedef std::shared_ptr<M4 const> M4ConstPtr;
  typedef std::shared_ptr<M5 const> M5ConstPtr;
  typedef std::shared_ptr<M6 const> M6ConstPtr;
  typedef std::shared_ptr<M7 const> M7ConstPtr;
  typedef std::shared_ptr<M8 const> M8ConstPtr;
  typedef const std::shared_ptr<NullType const>& NullP;

  template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
  Connection addCallback(const std::function<void(P0, P1, P2, P3, P4, P5, P6, P7, P8)>& callback)
  {
    CallbackHelper9T<P0, P1, P2, P3, P4, P5, P6, P7, P8>* helper = new CallbackHelper9T<P0, P1, P2, P3, P4, P5, P6, P7, P8>(callback);

    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_.push_back(CallbackHelper9Ptr(helper));
    return Connection(std::bind(&Signal9::removeCallback, this, callbacks_.back()));
  }

  template<typename P0, typename P1>
  Connection addCallback(void(*callback)(P0, P1))
  {
    return addCallback(std::function<void(P0, P1, NullP, NullP, NullP, NullP, NullP, NullP, NullP)>(std::bind(callback, _1, _2)));
  }

  template<typename P0, typename P1, typename P2>
  Connection addCallback(void(*callback)(P0, P1, P2))
  {
    return addCallback(std::function<void(P0, P1, P2, NullP, NullP, NullP, NullP, NullP, NullP)>(std::bind(callback, _1, _2, _3)));
  }

  template<typename P0, typename P1, typename P2, typename P3>
  Connection addCallback(void(*callback)(P0, P1, P2, P3))
  {
    return addCallback(std::function<void(P0, P1, P2, P3, NullP, NullP, NullP, NullP, NullP)>(std::bind(callback, _1, _2, _3, _4)));
  }

  template<typename P0, typename P1, typename P2, typename P3, typename P4>
  Connection addCallback(void(*callback)(P0, P1, P2, P3, P4))
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, NullP, NullP, NullP, NullP)>(std::bind(callback, _1, _2, _3, _4, _5)));
  }

  template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
  Connection addCallback(void(*callback)(P0, P1, P2, P3, P4, P5))
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, NullP, NullP, NullP)>(std::bind(callback, _1, _2, _3, _4, _5, _6)));
  }

  template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
  Connection addCallback(void(*callback)(P0, P1, P2, P3, P4, P5, P6))
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, P6, NullP, NullP)>(std::bind(callback, _1, _2, _3, _4, _5, _6, _7)));
  }

  template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
  Connection addCallback(void(*callback)(P0, P1, P2, P3, P4, P5, P6, P7))
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, P6, P7, NullP)>(std::bind(callback, _1, _2, _3, _4, _5, _6, _7, _8)));
  }

  template<typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
  Connection addCallback(void(*callback)(P0, P1, P2, P3, P4, P5, P6, P7, P8))
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, P6, P7, P8)>(std::bind(callback, _1, _2, _3, _4, _5, _6, _7, _8, _9)));
  }

  template<typename T, typename P0, typename P1>
  Connection addCallback(void(T::*callback)(P0, P1), T* t)
  {
    return addCallback(std::function<void(P0, P1, NullP, NullP, NullP, NullP, NullP, NullP, NullP)>(std::bind(callback, t, _1, _2)));
  }

  template<typename T, typename P0, typename P1, typename P2>
  Connection addCallback(void(T::*callback)(P0, P1, P2), T* t)
  {
    return addCallback(std::function<void(P0, P1, P2, NullP, NullP, NullP, NullP, NullP, NullP)>(std::bind(callback, t, _1, _2, _3)));
  }

  template<typename T, typename P0, typename P1, typename P2, typename P3>
  Connection addCallback(void(T::*callback)(P0, P1, P2, P3), T* t)
  {
    return addCallback(std::function<void(P0, P1, P2, P3, NullP, NullP, NullP, NullP, NullP)>(std::bind(callback, t, _1, _2, _3, _4)));
  }

  template<typename T, typename P0, typename P1, typename P2, typename P3, typename P4>
  Connection addCallback(void(T::*callback)(P0, P1, P2, P3, P4), T* t)
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, NullP, NullP, NullP, NullP)>(std::bind(callback, t, _1, _2, _3, _4, _5)));
  }

  template<typename T, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
  Connection addCallback(void(T::*callback)(P0, P1, P2, P3, P4, P5), T* t)
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, NullP, NullP, NullP)>(std::bind(callback, t, _1, _2, _3, _4, _5, _6)));
  }

  template<typename T, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
  Connection addCallback(void(T::*callback)(P0, P1, P2, P3, P4, P5, P6), T* t)
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, P6, NullP, NullP)>(std::bind(callback, t, _1, _2, _3, _4, _5, _6, _7)));
  }

  template<typename T, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
  Connection addCallback(void(T::*callback)(P0, P1, P2, P3, P4, P5, P6, P7), T* t)
  {
    return addCallback(std::function<void(P0, P1, P2, P3, P4, P5, P6, P7, NullP)>(std::bind(callback, t, _1, _2, _3, _4, _5, _6, _7, _8)));
  }

  template<typename C>
  Connection addCallback( C& callback)
  {
    return addCallback<const M0ConstPtr&,
                     const M1ConstPtr&,
                     const M2ConstPtr&,
                     const M3ConstPtr&,
                     const M4ConstPtr&,
                     const M5ConstPtr&,
                     const M6ConstPtr&,
                     const M7ConstPtr&,
                     const M8ConstPtr&>(std::bind(callback, _1, _2, _3, _4, _5, _6, _7, _8, _9));
  }

  void removeCallback(const CallbackHelper9Ptr& helper)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    typename V_CallbackHelper9::iterator it = std::find(callbacks_.begin(), callbacks_.end(), helper);
    if (it != callbacks_.end())
    {
      callbacks_.erase(it);
    }
  }

  void call(const M0Event& e0, const M1Event& e1, const M2Event& e2, const M3Event& e3, const M4Event& e4,
            const M5Event& e5, const M6Event& e6, const M7Event& e7, const M8Event& e8)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    bool nonconst_force_copy = callbacks_.size() > 1;
    typename V_CallbackHelper9::iterator it = callbacks_.begin();
    typename V_CallbackHelper9::iterator end = callbacks_.end();
    for (; it != end; ++it)
    {
      const CallbackHelper9Ptr& helper = *it;
      helper->call(nonconst_force_copy, e0, e1, e2, e3, e4, e5, e6, e7, e8);
    }
  }

private:
  std::mutex mutex_;
  V_CallbackHelper9 callbacks_;
};

} // message_filters

#endif // MESSAGE_FILTERS_SIGNAL9_H


