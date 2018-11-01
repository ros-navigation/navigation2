/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSLIB_MESSAGE_TRAITS_H
#define ROSLIB_MESSAGE_TRAITS_H

#include <type_traits>
#include <rclcpp/rclcpp.hpp>

namespace message_filters
{
namespace message_traits
{

/**
 * \brief HasHeader informs whether or not there is a header that gets serialized as the first thing in the message
 */
template<typename M, typename = void> struct HasHeader : public std::false_type {};

template <typename M>
struct HasHeader<M, decltype((void) M::header)> : std::true_type {};

/**
 * \brief TimeStamp trait.  In the default implementation pointer()
 * returns &m.header.stamp if HasHeader<M>::value is true, otherwise returns NULL.  value()
 * does not exist, and causes a compile error
 */
template<typename M, typename Enable = void>
struct TimeStamp
{
  static rclcpp::Time value(const M& m) {
    (void)m;
    return rclcpp::Time();
  }
};

template<typename M>
struct TimeStamp<M, typename std::enable_if<HasHeader<M>::value>::type >
{
  static rclcpp::Time value(const M& m) {
    auto stamp = m.header.stamp;
    return rclcpp::Time(stamp.sec, stamp.nanosec);
  }
};

} // namespace message_traits
} // namespace message_filters

#endif // ROSLIB_MESSAGE_TRAITS_H
