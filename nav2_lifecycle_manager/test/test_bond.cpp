// Copyright (c) 2020 Samsung Research America
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

#include <memory>
#include <gtest/gtest.h>

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"

// class holding lifecycle node and a bond server
// fns for break, check if broken
// and another with just a normal node to see if it works that way..


// shim to make it on instantiation

// test A
  // make bond connection to it
  // make sure good
  // wait 5 seconds and make sure still good
  // have this break it and check that is broken

// test B
  // make bond connection to it
  // make sure good
  // have that break it and check that this is broken

// test C
  // make bond connection to it
  // make sure good
  // delete that object
  // make sure goes down

// test D
  // set timeout to 0
  // check no connection
