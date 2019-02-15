// Copyright (c) 2019 Intel Corporation
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

using std::chrono::high_resolution_clock;
using std::to_string;
using std::string;
using std::replace_if;
using std::isalnum;

namespace nav2_util
{

string sanitizeNodeName(const string & potential_node_name)
{
  string node_name(potential_node_name);
  // read this as `replace` characters in `node_name` `if` not alphanumeric.
  // replace with '_'
  replace_if(begin(node_name), end(node_name),
    [](auto c) {return !isalnum(c);},
    '_');
  return node_name;
}

std::string timeToString(size_t len)
{
  string output(len, '0');  // prefill the string with zeros
  auto timepoint = high_resolution_clock::now();
  auto timecount = timepoint.time_since_epoch().count();
  auto timestring = to_string(timecount);
  if (timestring.length() >= len) {
    // if `timestring` is shorter, put it at the end of `output`
    output.replace(0, len,
      timestring,
      timestring.length() - len, len);
  } else {
    // if `output` is shorter, just copy in the end of `timestring`
    output.replace(len - timestring.length(), timestring.length(),
      timestring,
      0, timestring.length());
  }
  return output;
}

std::string generateInternalNodeName(const std::string & prefix)
{
  return sanitizeNodeName(prefix) + "_" + timeToString(8);
}

rclcpp::Node::SharedPtr generateInternalNode(const std::string & prefix)
{
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  return rclcpp::Node::make_shared(generateInternalNodeName(prefix));
}

}  // namespace nav2_util
