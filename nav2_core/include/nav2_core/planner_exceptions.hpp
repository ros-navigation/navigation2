/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  Copyright (c) 2019, Intel Corporation
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV2_CORE__PLANNER_EXCEPTIONS_HPP_
#define NAV2_CORE__PLANNER_EXCEPTIONS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <memory>

#include "nav2_msgs/action/compute_path_through_poses.hpp"

namespace nav2_core
{

using ComputePathThroughPosesResult = nav2_msgs::action::ComputePathThroughPoses::Result;

class PlannerException : public std::runtime_error
{
public:
  explicit PlannerException(
    const std::string & description,
    uint16_t error_code = ComputePathThroughPosesResult::UNKNOWN)
  : std::runtime_error(description), error_code_(error_code) {}

  uint16_t errorCode() const {return error_code_;}

private:
  uint16_t error_code_;
};

class InvalidPlanner : public PlannerException
{
public:
  explicit InvalidPlanner(const std::string & description)
  : PlannerException(description, ComputePathThroughPosesResult::INVALID_PLANNER) {}
};

class StartOccupied : public PlannerException
{
public:
  explicit StartOccupied(const std::string & description)
  : PlannerException(description, ComputePathThroughPosesResult::START_OCCUPIED) {}
};

class GoalOccupied : public PlannerException
{
public:
  explicit GoalOccupied(const std::string & description)
  : PlannerException(description, ComputePathThroughPosesResult::GOAL_OCCUPIED) {}
};

class StartOutsideMapBounds : public PlannerException
{
public:
  explicit StartOutsideMapBounds(const std::string & description)
  : PlannerException(description, ComputePathThroughPosesResult::START_OUTSIDE_MAP) {}
};

class GoalOutsideMapBounds : public PlannerException
{
public:
  explicit GoalOutsideMapBounds(const std::string & description)
  : PlannerException(description, ComputePathThroughPosesResult::GOAL_OUTSIDE_MAP) {}
};

class NoValidPathCouldBeFound : public PlannerException
{
public:
  explicit NoValidPathCouldBeFound(const std::string & description)
  : PlannerException(description, ComputePathThroughPosesResult::NO_VALID_PATH) {}
};

class PlannerTimedOut : public PlannerException
{
public:
  explicit PlannerTimedOut(const std::string & description)
  : PlannerException(description, ComputePathThroughPosesResult::TIMEOUT) {}
};

class PlannerTFError : public PlannerException
{
public:
  explicit PlannerTFError(const std::string & description)
  : PlannerException(description, ComputePathThroughPosesResult::TF_ERROR) {}
};

class NoViapointsGiven : public PlannerException
{
public:
  explicit NoViapointsGiven(const std::string & description)
  : PlannerException(description, ComputePathThroughPosesResult::NO_VIAPOINTS_GIVEN) {}
};

class PlannerCancelled : public PlannerException
{
public:
  explicit PlannerCancelled(const std::string & description)
  : PlannerException(description) {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__PLANNER_EXCEPTIONS_HPP_
