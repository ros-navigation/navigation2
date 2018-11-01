#include <gtest/gtest.h>
#include "action_test_node.h"
#include "condition_test_node.h"
#include "behavior_tree_core/xml_parsing.h"
#include "../sample_nodes/crossdoor_nodes.h"

// clang-format off

const std::string xml_text = R"(

<root main_tree_to_execute = "MainTree" >

    <BehaviorTree ID="MainTree">
        <Fallback name="root_selector">

            <Sequence name="door_open_sequence">
                <Action ID="IsDoorOpen" />
                <Action ID="PassThroughDoor" />
            </Sequence>

            <Sequence name="door_closed_sequence">
                <Decorator ID="Inverter">
                     <Action ID="IsDoorOpen" />
                </Decorator>
                <Action ID="OpenDoor" />
                <Action ID="PassThroughDoor" />
                <Action ID="CloseDoor" />
            </Sequence>

            <Action ID="PassThroughWindow" />

        </Fallback>
    </BehaviorTree>

    <!-- TreeNodesModel is used only by the Graphic interface -->
    <TreeNodesModel>
        <Action ID="IsDoorOpen" />
        <Action ID="PassThroughDoor" />
        <Action ID="CloseDoor" />
        <Action ID="OpenDoor" />
        <Action ID="PassThroughWindow" />
        <Decorator ID="Invert" />
        <Decorator ID="RetryUntilSuccesful">
            <Parameter label="num_attempts" type="Int" />
        </Decorator>
        <Decorator ID="Repeat">
            <Parameter label="num_cycles" type="Int" />
        </Decorator>
    </TreeNodesModel>
</root>
        )";

const std::string xml_text_subtree = R"(

<root main_tree_to_execute = "MainTree" >

  <BehaviorTree ID="CrossDoorSubtree">
    <Sequence name="door_sequence">
      <Decorator ID="Inverter">
        <Action ID="IsDoorLocked" />
      </Decorator>
      <Action ID="OpenDoor" />
      <Action ID="PassThroughDoor" />
      <Action ID="CloseDoor" />
    </Sequence>
  </BehaviorTree>

  <!-- This tree will include the other one -->
  <BehaviorTree ID="MainTree">
    <Fallback name="root_selector">
      <SubTree ID="CrossDoorSubtree" />
      <Action ID="PassThroughWindow" />
    </Fallback>
  </BehaviorTree>

</root>
        )";
// clang-format on

TEST(BehaviorTreeFactory, VerifyLargeTree)
{
    BT::BehaviorTreeFactory factory;
    CrossDoor::RegisterNodes(factory);

    BT::XMLParser parser(factory);
    parser.loadFromText(xml_text);

    std::vector<std::string> errors;
    bool res = parser.verifyXML(errors);

    for (const std::string& str : errors)
    {
        std::cout << str << std::endl;
    }

    ASSERT_EQ(res, true);
    ASSERT_EQ(errors.size(), 0);

    std::vector<BT::TreeNode::Ptr> nodes;

    BT::TreeNode::Ptr root_node = parser.instantiateTree(nodes);

    BT::printTreeRecursively(root_node.get());

    ASSERT_EQ(root_node->name(), "root_selector");

    auto fallback = dynamic_cast<const BT::FallbackNode*>(root_node.get());
    ASSERT_TRUE(fallback != nullptr);

    ASSERT_EQ(fallback->children().size(), 3);
    ASSERT_EQ(fallback->child(0)->name(), "door_open_sequence");
    ASSERT_EQ(fallback->child(1)->name(), "door_closed_sequence");
    ASSERT_EQ(fallback->child(2)->name(), "PassThroughWindow");

    auto sequence_open = dynamic_cast<const BT::SequenceNode*>(fallback->child(0));
    ASSERT_TRUE(sequence_open != nullptr);

    ASSERT_EQ(sequence_open->children().size(), 2);
    ASSERT_EQ(sequence_open->child(0)->name(), "IsDoorOpen");
    ASSERT_EQ(sequence_open->child(1)->name(), "PassThroughDoor");

    auto sequence_closed = dynamic_cast<const BT::SequenceNode*>(fallback->child(1));
    ASSERT_TRUE(sequence_closed != nullptr);

    ASSERT_EQ(sequence_closed->children().size(), 4);
    ASSERT_EQ(sequence_closed->child(0)->name(), "Inverter");
    ASSERT_EQ(sequence_closed->child(1)->name(), "OpenDoor");
    ASSERT_EQ(sequence_closed->child(2)->name(), "PassThroughDoor");
    ASSERT_EQ(sequence_closed->child(3)->name(), "CloseDoor");

    auto decorator = dynamic_cast<const BT::InverterNode*>(sequence_closed->child(0));
    ASSERT_TRUE(decorator != nullptr);

    ASSERT_EQ(decorator->child()->name(), "IsDoorOpen");
}

TEST(BehaviorTreeFactory, Subtree)
{
    BT::BehaviorTreeFactory factory;
    CrossDoor::RegisterNodes(factory);

    BT::XMLParser parser(factory);
    parser.loadFromText(xml_text_subtree);

    std::vector<std::string> errors;
    bool res = parser.verifyXML(errors);

    for (const std::string& str : errors)
    {
        std::cout << str << std::endl;
    }

    ASSERT_EQ(res, true);
    ASSERT_EQ(errors.size(), 0);

    std::vector<BT::TreeNode::Ptr> nodes;

    BT::TreeNode::Ptr root_node = parser.instantiateTree(nodes);
    BT::printTreeRecursively(root_node.get());

    ASSERT_EQ(root_node->name(), "root_selector");

    auto root_selector = dynamic_cast<const BT::FallbackNode*>(root_node.get());
    ASSERT_TRUE(root_selector != nullptr);
    ASSERT_EQ(root_selector->children().size(), 2);
    ASSERT_EQ(root_selector->child(0)->name(), "CrossDoorSubtree");
    ASSERT_EQ(root_selector->child(1)->name(), "PassThroughWindow");

    auto subtree = dynamic_cast<const BT::DecoratorSubtreeNode*>(root_selector->child(0));
    ASSERT_TRUE(subtree != nullptr);

    auto sequence = dynamic_cast<const BT::SequenceNode*>(subtree->child());
    ASSERT_TRUE(sequence != nullptr);

    ASSERT_EQ(sequence->children().size(), 4);
    ASSERT_EQ(sequence->child(0)->name(), "Inverter");
    ASSERT_EQ(sequence->child(1)->name(), "OpenDoor");
    ASSERT_EQ(sequence->child(2)->name(), "PassThroughDoor");
    ASSERT_EQ(sequence->child(3)->name(), "CloseDoor");

    auto decorator = dynamic_cast<const BT::InverterNode*>(sequence->child(0));
    ASSERT_TRUE(decorator != nullptr);

    ASSERT_EQ(decorator->child()->name(), "IsDoorLocked");
}
