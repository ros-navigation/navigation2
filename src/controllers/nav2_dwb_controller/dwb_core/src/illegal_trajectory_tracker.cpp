/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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

#include "dwb_core/illegal_trajectory_tracker.hpp"
#include <map>
#include <utility>
#include <string>
#include <sstream>

namespace dwb_core
{
void IllegalTrajectoryTracker::addIllegalTrajectory(
  const dwb_core::IllegalTrajectoryException & e)
{
  counts_[std::make_pair(e.getCriticName(), e.what())]++;
  illegal_count_++;
}

void IllegalTrajectoryTracker::addLegalTrajectory()
{
  legal_count_++;
}

std::map<std::pair<std::string, std::string>,
  double> IllegalTrajectoryTracker::getPercentages() const
{
  std::map<std::pair<std::string, std::string>, double> percents;
  double denominator = static_cast<double>(legal_count_ + illegal_count_);
  for (auto const & x : counts_) {
    percents[x.first] = static_cast<double>(x.second) / denominator;
  }
  return percents;
}

std::string IllegalTrajectoryTracker::getMessage() const
{
  std::ostringstream msg;
  if (legal_count_ == 0) {
    msg << "No valid trajectories out of " << illegal_count_ << "! ";
  } else {
    unsigned int total = legal_count_ + illegal_count_;
    msg << legal_count_ << " valid trajectories found (";
    msg << static_cast<double>(100 * legal_count_) / static_cast<double>(total);
    msg << "% of " << total << "). ";
  }
  return msg.str();
}

}  // namespace dwb_core
