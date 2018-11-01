#ifndef BT_FLATBUFFER_HELPER_H
#define BT_FLATBUFFER_HELPER_H

#include "abstract_logger.h"
#include "BT_logger_generated.h"

namespace BT
{
inline BT_Serialization::Type convertToFlatbuffers(NodeType type)
{
    switch (type)
    {
        case BT::NodeType::ACTION:
            return BT_Serialization::Type::ACTION;
        case BT::NodeType::DECORATOR:
            return BT_Serialization::Type::DECORATOR;
        case BT::NodeType::CONTROL:
            return BT_Serialization::Type::CONTROL;
        case BT::NodeType::CONDITION:
            return BT_Serialization::Type::CONDITION;
        case BT::NodeType::SUBTREE:
            return BT_Serialization::Type::SUBTREE;
        case BT::NodeType::UNDEFINED:
            return BT_Serialization::Type::UNDEFINED;
    }
    return BT_Serialization::Type::UNDEFINED;
}

inline BT_Serialization::Status convertToFlatbuffers(NodeStatus type)
{
    switch (type)
    {
        case BT::NodeStatus::IDLE:
            return BT_Serialization::Status::IDLE;
        case BT::NodeStatus::SUCCESS:
            return BT_Serialization::Status::SUCCESS;
        case BT::NodeStatus::RUNNING:
            return BT_Serialization::Status::RUNNING;
        case BT::NodeStatus::FAILURE:
            return BT_Serialization::Status::FAILURE;
    }
    return BT_Serialization::Status::IDLE;
}

inline void CreateFlatbuffersBehaviorTree(flatbuffers::FlatBufferBuilder& builder,
                                          BT::TreeNode* root_node)
{
    std::vector<flatbuffers::Offset<BT_Serialization::TreeNode>> fb_nodes;

    applyRecursiveVisitor(root_node, [&](BT::TreeNode* node) {
        std::vector<uint16_t> children_uid;
        if (auto control = dynamic_cast<BT::ControlNode*>(node))
        {
            children_uid.reserve(control->children().size());
            for (const auto& child : control->children())
            {
                children_uid.push_back(child->UID());
            }
        }
        else if (auto decorator = dynamic_cast<BT::DecoratorNode*>(node))
        {
            const auto& child = decorator->child();
            children_uid.push_back(child->UID());
        }

        std::vector<flatbuffers::Offset<BT_Serialization::KeyValue>> params;
        const NodeParameters& init_params = node->initializationParameters();
        for (const auto& it : init_params)
        {
            params.push_back(BT_Serialization::CreateKeyValueDirect(builder, it.first.c_str(),
                                                                    it.second.c_str()));
        }

        auto tn = BT_Serialization::CreateTreeNode(
            builder, node->UID(), builder.CreateVector(children_uid),
            convertToFlatbuffers(node->type()), convertToFlatbuffers(node->status()),
            builder.CreateString(node->name().c_str()),
            builder.CreateString(node->registrationName().c_str()), builder.CreateVector(params));

        fb_nodes.push_back(tn);
    });

    auto behavior_tree = BT_Serialization::CreateBehaviorTree(builder, root_node->UID(),
                                                              builder.CreateVector(fb_nodes));

    builder.Finish(behavior_tree);
}

/** Serialize manually the informations about state transition
 * No flatbuffer serialization here
 */
inline std::array<uint8_t, 12> SerializeTransition(uint16_t UID, Duration timestamp,
                                                   NodeStatus prev_status, NodeStatus status)
{
    using namespace std::chrono;
    std::array<uint8_t, 12> buffer;
    auto usec = duration_cast<microseconds>(timestamp).count();
    uint32_t t_sec = usec / 1000000;
    uint32_t t_usec = usec % 1000000;

    flatbuffers::WriteScalar(&buffer[0], t_sec);
    flatbuffers::WriteScalar(&buffer[4], t_usec);
    flatbuffers::WriteScalar(&buffer[8], UID);

    flatbuffers::WriteScalar(&buffer[10], static_cast<int8_t>(convertToFlatbuffers(prev_status)));
    flatbuffers::WriteScalar(&buffer[11], static_cast<int8_t>(convertToFlatbuffers(status)));

    return buffer;
}

}   // end namespace

#endif   // BT_FLATBUFFER_HELPER_H
