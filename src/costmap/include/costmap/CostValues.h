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

#ifndef COSTMAP__COSTVALUES_H_
#define COSTMAP__COSTVALUES_H_

/** Provides a mapping for often used cost values */
enum class CostValue: uint8_t
{
  no_information = 255,
  lethal_obstacle = 254,
  inscribed_inflated_obstacle = 253,
  medium_cost = 128,
  free_space = 0
};
#endif  // COSTMAP__COSTVALUES_H_
