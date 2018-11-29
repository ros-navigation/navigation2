/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_amcl/amcl_node.hpp"

std::shared_ptr<rclcpp::Clock> simtime_clock;

int
main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // TODO(crdelsey): This is workaround for Issue #339
  // Depending on a persons particular setup, most of the time when a laser
  // scan message is received, the transform for it is not available yet. With
  // the previous implementation, AMCL would discard that data. On some machines
  // we could go for thousands of scans without a successful transform resulting
  // in basically an unusable system. Lacking message filters, the fix for this
  // is to enable transform timeouts, however, at the moment, transform timeouts
  // hang when use_sim_time is active. This workaround creates a seperate node
  // for the sole purpose of provide a clock that can keep updating when the
  // transform code is in a tight loop waiting for transform data or the timeout.
  rclcpp::executors::MultiThreadedExecutor exec;
  auto clockNode = rclcpp::Node::make_shared("amcl_clock_node");
  simtime_clock = clockNode->get_clock();
  auto amclNode = std::make_shared<AmclNode>();
  exec.add_node(clockNode);
  exec.add_node(amclNode);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
