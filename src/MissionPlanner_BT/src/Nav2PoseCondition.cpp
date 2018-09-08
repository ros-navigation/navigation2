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

#include "Nav2PoseCondition.h"

Nav2PoseCondition::Nav2PoseCondition(std::string name) : BT::ConditionNode::ConditionNode(name)
{
    type_ = BT::CONDITION_NODE;
    boolean_value_ = true;
}

Nav2PoseCondition::~Nav2PoseCondition() {}

BT::ReturnStatus Nav2PoseCondition::Tick()
{
    if (boolean_value_)
    {
        set_status(BT::SUCCESS);
        return BT::SUCCESS;
    }
    else
    {
        set_status(BT::FAILURE);
        return BT::FAILURE;
    }
}

void Nav2PoseCondition::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}