/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_COSTMAP_2D_PUBLISHER_H_
#define COSTMAP_2D_COSTMAP_2D_PUBLISHER_H_
#include <rclcpp/rclcpp.hpp>
#include <costmap_2d/costmap_2d.h>

namespace costmap_2d
{
/**
 * @class Costmap2DPublisher
 * @brief A tool to periodically publish visualization data from a Costmap2D
 */
class Costmap2DPublisher
{
public:
  /**
   * @brief  Constructor for the Costmap2DPublisher
   */
  Costmap2DPublisher(
    rclcpp::Node::SharedPtr ros_node, Costmap2D * costmap, std::string global_frame,
    std::string topic_name, bool always_send_full_costmap = false) {}

  /**
   * @brief  Destructor
   */
  ~Costmap2DPublisher() {}

  /** @brief Include the given bounds in the changed-rectangle. */
  void updateBounds(unsigned int x0, unsigned int xn, unsigned int y0, unsigned int yn) {}

  /**
   * @brief  Publishes the visualization data over ROS
   */
  void publishCostmap();

  /**
   * @brief Check if the publisher is active
   * @return True if the frequency for the publisher is non-zero, false otherwise
   */
  bool active()
  {
    return false;
  }

};
}  // namespace costmap_2d
#endif  // COSTMAP_2D_COSTMAP_2D_PUBLISHER_H
