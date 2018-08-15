// Copyright (c) 2018 Intel Corporation
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

#ifndef PLANNING__LOGGER_H_
#define PLANNING__LOGGER_H_

#include <cstdio>

// TODO: Figure out how to get a reference to the node logger
#define ROS_ERROR(...) std::fprintf(stderr, __VA_ARGS__)

#define ROS_WARN(...) std::fprintf(stderr, __VA_ARGS__)

#define ROS_INFO(...) std::fprintf(stderr, __VA_ARGS__)

#define ROS_DEBUG(...) std::fprintf(stderr, __VA_ARGS__)

#define ROS_DEBUG_NAMED(...) std::fprintf(stderr, __VA_ARGS__)

#define ROS_INFO_ONCE(...) std::fprintf(stderr, __VA_ARGS__)

#endif // PLANNING__LOGGER_H_
