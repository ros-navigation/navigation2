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

#ifndef MESSAGE_FILTERS_NULL_TYPES_H
#define MESSAGE_FILTERS_NULL_TYPES_H

#include "connection.h"

#include "message_filters/message_traits.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace message_filters
{
struct MsgHeader
{
  rclcpp::Time stamp;
};


struct NullType
{
  static MsgHeader header;
};

typedef std::shared_ptr<NullType const> NullTypeConstPtr;
template<class M>
struct NullFilter
{
  template<typename C>
  Connection registerCallback(const C&)
  {
    return Connection();
  }

  template<typename P>
  Connection registerCallback(const std::function<void(P)>&)
  {
    return Connection();
  }
};

namespace message_traits
{
template<>
struct TimeStamp<message_filters::NullType>
{
  static rclcpp::Time value(const message_filters::NullType&)
  {
    return rclcpp::Time();
  }
};
}
}

#endif // MESSAGE_FILTERS_NULL_TYPES_H
