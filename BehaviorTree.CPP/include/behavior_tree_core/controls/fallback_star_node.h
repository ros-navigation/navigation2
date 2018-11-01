/* Copyright (C) 2015-2018 Michele Colledanchise -  All Rights Reserved
 * Copyright (C) 2018 Davide Faconti -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef selector_node_WITH_MEMORY_H
#define selector_node_WITH_MEMORY_H

#include "behavior_tree_core/control_node.h"

namespace BT
{
/**
 * @brief The FallbackStarNode is used to try different strategies,
 * until one succeed.
 * If any child returns RUNNING, previous children will NOT be ticked again.
 *
 * - If all the children return FAILURE, this node returns FAILURE.
 *
 * - If a child returns RUNNING, this node returns RUNNING.
 *   Loop is NOT restarted, the same running child will be ticked again.
 *
 * - If a child returns SUCCESS, stop the loop and returns SUCCESS.
 */

class FallbackStarNode : public ControlNode
{
  public:
    FallbackStarNode(const std::string& name);

    virtual void halt() override;

  private:
    unsigned int current_child_idx_;

    virtual BT::NodeStatus tick() override;
};
}

#endif   // selector_node_WITH_MEMORY_H
