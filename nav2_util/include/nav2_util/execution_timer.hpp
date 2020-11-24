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

#ifndef NAV2_UTIL__EXECUTION_TIMER_HPP_
#define NAV2_UTIL__EXECUTION_TIMER_HPP_

#include <chrono>

namespace nav2_util
{

/// @brief Measures execution time of code between calls to start and end
class ExecutionTimer
{
public:
  using Clock = std::chrono::high_resolution_clock;
  using nanoseconds = std::chrono::nanoseconds;

  /// @brief Call just prior to code you want to measure
  void start() {start_ = Clock::now();}

  /// @brief Call just after the code you want to measure
  void end() {end_ = Clock::now();}

  /// @brief Extract the measured time as an integral std::chrono::duration object
  nanoseconds elapsed_time() {return end_ - start_;}

  /// @brief Extract the measured time as a floating point number of seconds.
  double elapsed_time_in_seconds()
  {
    return std::chrono::duration<double>(end_ - start_).count();
  }

protected:
  Clock::time_point start_;
  Clock::time_point end_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__EXECUTION_TIMER_HPP_
