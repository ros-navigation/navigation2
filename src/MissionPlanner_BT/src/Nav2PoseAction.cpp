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

#include "Nav2PoseAction.h"

Nav2PoseAction::Nav2PoseAction(std::string name) : ActionNode::ActionNode(name)
{
}

BT::ReturnStatus Nav2PoseAction::Tick()
{
    if (is_halted())
    {
        return BT::HALTED;
    }
    int count = 0;
    while (count != 10)
    {
        count++;
        std::cout << "Navigating to pose.. " << count << std::endl;
        if (count == 5)
        {
            count = 0;
            std::cout << "Navigating to pose has succeeded!" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return BT::SUCCESS;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        //else
        //return BT::RUNNING;
    }
}

void Nav2PoseAction::Halt()
{
    std::cout << "Nav2Pose Action is halted!" << std::endl;
}
