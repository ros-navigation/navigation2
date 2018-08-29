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

#include "PlannerAction.h"

PlannerAction::PlannerAction(std::string name) : ActionNode::ActionNode(name)
{
}

BT::ReturnStatus PlannerAction::Tick()
{
    if (is_halted())
    {
        return BT::HALTED;
    }
    int count = 0;
    while (count != 10)
    {
        count++;
        std::cout << "Canceling the mission... " << count << std::endl;
        if (count == 2)
        {
            count = 0;
            std::cout << "Mission is canceled!" << std::endl;
            return BT::SUCCESS;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void PlannerAction::Halt()
{
    std::cout << "Cancel Action is halted!" << std::endl;
}
