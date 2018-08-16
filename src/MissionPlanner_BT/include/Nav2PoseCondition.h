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

#include <behavior_tree.h>

class Nav2PoseCondition : public BT::ConditionNode
{
public:
  Nav2PoseCondition(std::string name);
  ~Nav2PoseCondition();
  void set_boolean_value(bool boolean_value);
  BT::ReturnStatus Tick();

private:
  bool boolean_value_;
};
