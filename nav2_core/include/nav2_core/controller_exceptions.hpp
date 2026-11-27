// Copyright (c) 2022. Joshua Wallace
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef NAV2_CORE__CONTROLLER_EXCEPTIONS_HPP_
#define NAV2_CORE__CONTROLLER_EXCEPTIONS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>

#include "nav2_msgs/action/follow_path.hpp"

namespace nav2_core
{

using FollowPathResult = nav2_msgs::action::FollowPath::Result;

/**
 * @class ControllerException
 * @brief Base controller exception. It carries the FollowPath error code
 * that the controller server reports in the action result.
 * Throw it directly with any FollowPath error code to abort the goal.
 */
class ControllerException : public std::runtime_error
{
public:
  explicit ControllerException(
    const std::string & description,
    uint16_t error_code = FollowPathResult::UNKNOWN)
  : std::runtime_error(description), error_code_(error_code) {}

  uint16_t errorCode() const {return error_code_;}

private:
  uint16_t error_code_;
};

class InvalidController : public ControllerException
{
public:
  explicit InvalidController(const std::string & description)
  : ControllerException(description, FollowPathResult::INVALID_CONTROLLER) {}
};

class ControllerTFError : public ControllerException
{
public:
  explicit ControllerTFError(const std::string & description)
  : ControllerException(description, FollowPathResult::TF_ERROR) {}
};

class FailedToMakeProgress : public ControllerException
{
public:
  explicit FailedToMakeProgress(const std::string & description)
  : ControllerException(description, FollowPathResult::FAILED_TO_MAKE_PROGRESS) {}
};

class PatienceExceeded : public ControllerException
{
public:
  explicit PatienceExceeded(const std::string & description)
  : ControllerException(description, FollowPathResult::PATIENCE_EXCEEDED) {}
};

class InvalidPath : public ControllerException
{
public:
  explicit InvalidPath(const std::string & description)
  : ControllerException(description, FollowPathResult::INVALID_PATH) {}
};

/**
 * @class NoValidControl
 * @brief The controller found no valid command this cycle. The controller server
 * retries within failure_tolerance before it aborts the goal with this error code.
 * Pass a more specific FollowPath error code to report the cause.
 */
class NoValidControl : public ControllerException
{
public:
  explicit NoValidControl(
    const std::string & description,
    uint16_t error_code = FollowPathResult::NO_VALID_CONTROL)
  : ControllerException(description, error_code) {}
};

class ControllerTimedOut : public ControllerException
{
public:
  explicit ControllerTimedOut(const std::string & description)
  : ControllerException(description, FollowPathResult::CONTROLLER_TIMED_OUT) {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__CONTROLLER_EXCEPTIONS_HPP_
