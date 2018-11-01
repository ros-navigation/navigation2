#include "behavior_tree_logger/bt_file_logger.h"
#include "behavior_tree_logger/bt_flatbuffer_helper.h"

namespace BT
{
FileLogger::FileLogger(BT::TreeNode* root_node, const char* filename, uint16_t buffer_size)
  : StatusChangeLogger(root_node), buffer_max_size_(buffer_size)
{
    if (buffer_max_size_ != 0)
    {
        buffer_.reserve(buffer_max_size_);
    }

    enableTransitionToIdle(true);

    flatbuffers::FlatBufferBuilder builder(1024);
    CreateFlatbuffersBehaviorTree(builder, root_node);

    //-------------------------------------

    file_os_.open(filename, std::ofstream::binary | std::ofstream::out);

    // serialize the length of the buffer in the first 4 bytes
    char size_buff[4];
    flatbuffers::WriteScalar(size_buff, static_cast<int32_t>(builder.GetSize()));

    file_os_.write(size_buff, 4);
    file_os_.write(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
}

FileLogger::~FileLogger()
{
    this->flush();
    file_os_.close();
}

void FileLogger::callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                          NodeStatus status)
{
    std::array<uint8_t, 12> buffer =
        SerializeTransition(node.UID(), timestamp, prev_status, status);

    if (buffer_max_size_ == 0)
    {
        file_os_.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    }
    else
    {
        buffer_.push_back(buffer);
        if (buffer_.size() >= buffer_max_size_)
        {
            this->flush();
        }
    }
}

void FileLogger::flush()
{
    for (const auto& array : buffer_)
    {
        file_os_.write(reinterpret_cast<const char*>(array.data()), array.size());
    }
    file_os_.flush();
    buffer_.clear();
}
}
