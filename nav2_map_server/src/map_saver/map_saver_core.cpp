/* Copyright (c) 2020 Shivam Pandey pandeyshivam2017robotics@gmail.com
 * Copyright (c) 2020 Samsung Research Russia
 * Copyright 2019 Rover Robotics
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

//
// Created by shivam on 10/3/20.
//

#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_map_server/map_saver_core.hpp"
namespace nav2_map_server{

template<class mapT> MapSaver<mapT>::MapSaver() : nav2_util::LifecycleNode("map_saver"){

}

template<class mapT> MapSaver<mapT>::~MapSaver() = default;

template<class mapT>
nav2_util::CallbackReturn
MapSaver<mapT>::on_configure(const rclcpp_lifecycle::State &state) {
  return nav2_util::CallbackReturn::SUCCESS;
}

template<class mapT>
nav2_util::CallbackReturn
MapSaver<mapT>::on_activate(const rclcpp_lifecycle::State &state) {
  return nav2_util::CallbackReturn::SUCCESS;
}

template<class mapT>
nav2_util::CallbackReturn
MapSaver<mapT>::on_deactivate(const rclcpp_lifecycle::State &state) {
  return nav2_util::CallbackReturn::SUCCESS;
}

template<class mapT>
nav2_util::CallbackReturn
MapSaver<mapT>::on_cleanup(const rclcpp_lifecycle::State &state) {
  return nav2_util::CallbackReturn::SUCCESS;
}

template<class mapT>
nav2_util::CallbackReturn
MapSaver<mapT>::on_shutdown(const rclcpp_lifecycle::State &state) {
  return nav2_util::CallbackReturn::SUCCESS;

}

template<class mapT>
nav2_util::CallbackReturn
MapSaver<mapT>::on_error(const rclcpp_lifecycle::State &state) {
  return nav2_util::CallbackReturn::SUCCESS;

}
}
