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

#ifndef MESSAGE_FILTERS_CHAIN_H
#define MESSAGE_FILTERS_CHAIN_H

#include "simple_filter.h"
#include "pass_through.h"

#include <vector>

namespace message_filters
{
using namespace std::placeholders;
/**
 * \brief Base class for Chain, allows you to store multiple chains in the same container.  Provides filter retrieval
 * by index.
 */
class ChainBase
{
public:
  virtual ~ChainBase() {}

  /**
   * \brief Retrieve a filter from this chain by index.  Returns an empty shared_ptr if the index is greater than
   * the size of the chain.  \b NOT type-safe
   *
   * \param F [template] The type of the filter
   * \param index The index of the filter (returned by addFilter())
   */
  template<typename F>
  std::shared_ptr<F> getFilter(size_t index) const
  {
    std::shared_ptr<void> filter = getFilterForIndex(index);
    if (filter)
    {
      return std::static_pointer_cast<F>(filter);
    }

    return std::shared_ptr<F>();
  }

protected:
  virtual std::shared_ptr<void> getFilterForIndex(size_t index) const = 0;
};
typedef std::shared_ptr<ChainBase> ChainBasePtr;

/**
 * \brief Chains a dynamic number of simple filters together.  Allows retrieval of filters by index after they are added.
 *
 * The Chain filter provides a container for simple filters.  It allows you to store an N-long set of filters inside a single
 * structure, making it much easier to manage them.
 *
 * Adding filters to the chain is done by adding shared_ptrs of them to the filter.  They are automatically connected to each other
 * and the output of the last filter in the chain is forwarded to the callback you've registered with Chain::registerCallback
 *
 * Example:
\verbatim
void myCallback(const MsgConstPtr& msg)
{
}

Chain<Msg> c;
c.addFilter(std::make_shared<PassThrough<Msg> >());
c.addFilter(std::make_shared<PassThrough<Msg> >());
c.registerCallback(myCallback);
\endverbatim

 *
 * It is also possible to pass bare pointers in, which will not be automatically deleted when Chain is destructed:
\verbatim
Chain<Msg> c;
PassThrough<Msg> p;
c.addFilter(&p);
c.registerCallback(myCallback);
\endverbatim
 *
 */
template<typename M>
class Chain : public ChainBase, public SimpleFilter<M>
{
public:
  typedef std::shared_ptr<M const> MConstPtr;
  typedef MessageEvent<M const> EventType;

  /**
   * \brief Default constructor
   */
  Chain()
  {
  }

  /**
   * \brief Constructor with filter.  Calls connectInput(f)
   */
  template<typename F>
  Chain(F& f)
  {
    connectInput(f);
  }

  struct NullDeleter
  {
    void operator()(void const*) const
    {
    }
  };

  /**
   * \brief Add a filter to this chain, by bare pointer.  Returns the index of that filter in the chain.
   */
  template<class F>
  size_t addFilter(F* filter)
  {
    std::shared_ptr<F> ptr(filter, NullDeleter());
    return addFilter(ptr);
  }

  /**
   * \brief Add a filter to this chain, by shared_ptr.  Returns the index of that filter in the chain
   */
  template<class F>
  size_t addFilter(const std::shared_ptr<F>& filter)
  {
    FilterInfo info;
    info.add_func = std::bind((void(F::*)(const EventType&))&F::add, filter.get(), _1);
    info.filter = filter;
    info.passthrough = std::make_shared<PassThrough<M> >();

    last_filter_connection_.disconnect();
    info.passthrough->connectInput(*filter);
    last_filter_connection_ = info.passthrough->registerCallback(typename SimpleFilter<M>::EventCallback(std::bind(&Chain::lastFilterCB, this, _1)));
    if (!filters_.empty())
    {
      filter->connectInput(*filters_.back().passthrough);
    }

    uint32_t count = filters_.size();
    filters_.push_back(info);
    return count;
  }

  /**
   * \brief Retrieve a filter from this chain by index.  Returns an empty shared_ptr if the index is greater than
   * the size of the chain.  \b NOT type-safe
   *
   * \param F [template] The type of the filter
   * \param index The index of the filter (returned by addFilter())
   */
  template<typename F>
  std::shared_ptr<F> getFilter(size_t index) const
  {
    if (index >= filters_.size())
    {
      return std::shared_ptr<F>();
    }

    return std::static_pointer_cast<F>(filters_[index].filter);
  }

  /**
   * \brief Connect this filter's input to another filter's output.
   */
  template<class F>
  void connectInput(F& f)
  {
    incoming_connection_.disconnect();
    incoming_connection_ = f.registerCallback(typename SimpleFilter<M>::EventCallback(std::bind(&Chain::incomingCB, this, _1)));
  }

  /**
   * \brief Add a message to the start of this chain
   */
  void add(const MConstPtr& msg)
  {
    add(EventType(msg));
  }

  void add(const EventType& evt)
  {
    if (!filters_.empty())
    {
      filters_[0].add_func(evt);
    }
  }

protected:
  virtual std::shared_ptr<void> getFilterForIndex(size_t index) const
  {
    if (index >= filters_.size())
    {
      return std::shared_ptr<void>();
    }

    return filters_[index].filter;
  }

private:
  void incomingCB(const EventType& evt)
  {
    add(evt);
  }

  void lastFilterCB(const EventType& evt)
  {
    this->signalMessage(evt);
  }

  struct FilterInfo
  {
    std::function<void(const EventType&)> add_func;
    std::shared_ptr<void> filter;
    std::shared_ptr<PassThrough<M> > passthrough;
  };
  typedef std::vector<FilterInfo> V_FilterInfo;

  V_FilterInfo filters_;

  Connection incoming_connection_;
  Connection last_filter_connection_;
};
}

#endif // MESSAGE_FILTERS_CHAIN_H
