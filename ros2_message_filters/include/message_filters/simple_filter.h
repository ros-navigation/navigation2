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

#ifndef MESSAGE_FILTERS_SIMPLE_FILTER_H
#define MESSAGE_FILTERS_SIMPLE_FILTER_H
#include <functional>
#include <memory>
#include "connection.h"
#include "signal1.h"
#include "message_event.h"
#include <string>

namespace message_filters
{
using namespace std::placeholders;
/**
 * \brief Convenience base-class for simple filters which output a single message
 *
 * SimpleFilter provides some of the tricky callback registering functionality, so that
 * simple filters do not have to duplicate it.  It also provides getName()/setName() for debugging
 * purposes.
 */
template<class M>
class SimpleFilter : public noncopyable
{
public:
  typedef std::shared_ptr<M const> MConstPtr;
  typedef std::function<void(const MConstPtr&)> Callback;
  typedef MessageEvent<M const> EventType;
  typedef std::function<void(const EventType&)> EventCallback;

  /**
   * \brief Register a callback to be called when this filter has passed
   * \param callback The callback to call
   */
  template<typename C>
  Connection registerCallback(const C& callback)
  {
    typename CallbackHelper1<M>::Ptr helper = signal_.addCallback(Callback(callback));
    return Connection(std::bind(&Signal::removeCallback, &signal_, helper));
  }

  /**
   * \brief Register a callback to be called when this filter has passed
   * \param callback The callback to call
   */
  template<typename P>
  Connection registerCallback(const std::function<void(P)>& callback)
  {
    return Connection(std::bind(&Signal::removeCallback, &signal_, signal_.addCallback(callback)));
  }


  /**
   * \brief Register a callback to be called when this filter has passed
   * \param callback The callback to call
   */
  template<typename P>
  Connection registerCallback(void(*callback)(P))
  {
    typename CallbackHelper1<M>::Ptr helper = signal_.template addCallback<P>(std::bind(callback, _1));
    return Connection(std::bind(&Signal::removeCallback, &signal_, helper));
  }

  /**
   * \brief Register a callback to be called when this filter has passed
   * \param callback The callback to call
   */
  template<typename T, typename P>
  Connection registerCallback(void(T::*callback)(P), T* t)
  {
    typename CallbackHelper1<M>::Ptr helper = signal_.template addCallback<P>(std::bind(callback, t, _1));
    return Connection(std::bind(&Signal::removeCallback, &signal_, helper));
  }

  /**
   * \brief Set the name of this filter.  For debugging use.
   */
  void setName(const std::string& name) { name_ = name; }
  /**
   * \brief Get the name of this filter.  For debugging use.
   */
  const std::string& getName() { return name_; }

protected:
  /**
   * \brief Call all registered callbacks, passing them the specified message
   */
  void signalMessage(const MConstPtr& msg)
  {
    MessageEvent<M const> event(msg);

    signal_.call(event);
  }

  /**
   * \brief Call all registered callbacks, passing them the specified message
   */
  void signalMessage(const MessageEvent<M const>& event)
  {
    signal_.call(event);
  }

private:
  typedef Signal1<M> Signal;

  Signal signal_;

  std::string name_;
};

}

#endif

