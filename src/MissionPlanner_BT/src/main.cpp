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

#include <iostream>
#include <behavior_tree.h>

#include "CancelAction.h"
#include "Nav2PoseAction.h"
#include "TakePicAction.h"

#include "CancelCondition.h"
#include "Nav2PoseCondition.h"
#include "TakePicCondition.h"

int main(int argc, char *argv[])
{

    //Selector Nodes
    BT::FallbackNode *MissionPlanner = new BT::FallbackNode("MissionPlanner");

    //Sequence Nodes
    BT::SequenceNode *SequenceCancel = new BT::SequenceNode("Cancel");
    BT::SequenceNode *SequenceNav2Pose = new BT::SequenceNode("Nav2Pose");
    BT::SequenceNode *SequenceTakePic = new BT::SequenceNode("TakePic");

    //Condition Nodes
    CancelCondition *ConditionCancel = new CancelCondition("Cancel");
    Nav2PoseCondition *ConditionNav2Pose = new Nav2PoseCondition("Nav2Pose");
    TakePicCondition *ConditionTakePic = new TakePicCondition("TakePic");

    //Action Nodes
    CancelAction *ActionCancel = new CancelAction("Cancel");
    Nav2PoseAction *ActionNav2Pose = new Nav2PoseAction("Nav2Pose");
    TakePicAction *ActionTakePic = new TakePicAction("TakePic");

    int tick_period_milliseconds = 100;

    //MissionPlan
    ConditionCancel->set_boolean_value(false);
    ConditionNav2Pose->set_boolean_value(true);
    ConditionTakePic->set_boolean_value(false);

    MissionPlanner->AddChild(SequenceCancel);
    MissionPlanner->AddChild(SequenceNav2Pose);
    MissionPlanner->AddChild(SequenceTakePic);

    SequenceCancel->AddChild(ConditionCancel);
    SequenceCancel->AddChild(ActionCancel);

    SequenceNav2Pose->AddChild(ConditionNav2Pose);
    SequenceNav2Pose->AddChild(ActionNav2Pose);

    SequenceTakePic->AddChild(ConditionTakePic);
    SequenceTakePic->AddChild(ActionTakePic);

    Execute(MissionPlanner, tick_period_milliseconds);

    return 0;
}
