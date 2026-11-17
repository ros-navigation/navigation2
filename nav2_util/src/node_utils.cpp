// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_util/node_utils.hpp"
#include <chrono>
#include <string>
#include <algorithm>
#include <cctype>

#ifdef __APPLE__
  #include <pthread.h>
  #include <mach/mach.h>
  #include <mach/thread_policy.h>
#else
  #include <sched.h>
  #include <errno.h>
#endif

using std::chrono::high_resolution_clock;
using std::to_string;
using std::string;
using std::replace_if;
using std::isalnum;

namespace nav2_util
{

string sanitize_node_name(const string & potential_node_name)
{
  string node_name(potential_node_name);
  // read this as `replace` characters in `node_name` `if` not alphanumeric.
  // replace with '_'
  replace_if(
    begin(node_name), end(node_name),
    [](auto c) {return !isalnum(c);},
    '_');
  return node_name;
}

string add_namespaces(const string & top_ns, const string & sub_ns)
{
  if (!top_ns.empty() && top_ns.back() == '/') {
    if (top_ns.front() == '/') {
      return top_ns + sub_ns;
    } else {
      return "/" + top_ns + sub_ns;
    }
  }

  return top_ns + "/" + sub_ns;
}

std::string time_to_string(size_t len)
{
  string output(len, '0');  // prefill the string with zeros
  auto timepoint = high_resolution_clock::now();
  auto timecount = timepoint.time_since_epoch().count();
  auto timestring = to_string(timecount);
  if (timestring.length() >= len) {
    // if `timestring` is shorter, put it at the end of `output`
    output.replace(
      0, len,
      timestring,
      timestring.length() - len, len);
  } else {
    // if `output` is shorter, just copy in the end of `timestring`
    output.replace(
      len - timestring.length(), timestring.length(),
      timestring,
      0, timestring.length());
  }
  return output;
}

std::string generate_internal_node_name(const std::string & prefix)
{
  return sanitize_node_name(prefix) + "_" + time_to_string(8);
}

rclcpp::Node::SharedPtr generate_internal_node(const std::string & prefix)
{
  auto options =
    rclcpp::NodeOptions()
    .start_parameter_services(false)
    .start_parameter_event_publisher(false)
    .arguments({"--ros-args", "-r", "__node:=" + generate_internal_node_name(prefix), "--"});
  return rclcpp::Node::make_shared("_", options);
}

void setSoftRealTimePriority()
{
#ifdef __APPLE__
  // macOS: Use Mach thread API to approximate real-time scheduling
  thread_port_t thread = pthread_mach_thread_np(pthread_self());

  thread_time_constraint_policy_data_t policy;
  policy.period = 1000;       // in microseconds (1 kHz loop)
  policy.computation = 800;   // expected compute time per period
  policy.constraint = 1000;   // max latency
  policy.preemptible = 1;     // allow preemption by higher-priority threads

  kern_return_t result = thread_policy_set(
    thread,
    THREAD_TIME_CONSTRAINT_POLICY,
    (thread_policy_t)&policy,
    THREAD_TIME_CONSTRAINT_POLICY_COUNT
  );

  if (result != KERN_SUCCESS) {
    std::string errmsg =
      "Failed to set THREAD_TIME_CONSTRAINT_POLICY on macOS. "
      "Thread remains at default priority. Mach Error Code: " +
      std::to_string(result);
    throw std::runtime_error(errmsg);
  }
#else
  // Linux: True real-time scheduling (requires privileges)
  sched_param sch;
  sch.sched_priority = 49;
  if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
    std::string errmsg(
      "Cannot set as real-time thread. Users must set: <username> hard rtprio 99 and "
      "<username> soft rtprio 99 in /etc/security/limits.conf to enable "
      "realtime prioritization! Error: ");
    throw std::runtime_error(errmsg + std::strerror(errno));
  }
#endif
}

}  // namespace nav2_util
