#include "behavior_tree_logger/bt_zmq_publisher.h"
#include "behavior_tree_logger/bt_flatbuffer_helper.h"
#include <future>

namespace BT
{
std::atomic<bool> PublisherZMQ::ref_count(false);

void PublisherZMQ::createStatusBuffer()
{
    status_buffer_.clear();
    applyRecursiveVisitor(root_node_, [this](TreeNode* node) {
        size_t index = status_buffer_.size();
        status_buffer_.resize(index + 3);
        flatbuffers::WriteScalar<uint16_t>(&status_buffer_[index], node->UID());
        flatbuffers::WriteScalar<int8_t>(&status_buffer_[index + 2],
                                         static_cast<int8_t>(convertToFlatbuffers(node->status())));
    });
}

PublisherZMQ::PublisherZMQ(TreeNode* root_node, int max_msg_per_second)
  : StatusChangeLogger(root_node)
  , root_node_(root_node)
  , min_time_between_msgs_(std::chrono::microseconds(1000 * 1000) / max_msg_per_second)
  , zmq_context_(1)
  , zmq_publisher_(zmq_context_, ZMQ_PUB)
  , zmq_server_(zmq_context_, ZMQ_REP)
  , send_pending_(false)
{
    static bool first_instance = true;
    if (first_instance)
    {
        first_instance = false;
    }
    else
    {
        throw std::logic_error("Only one instance of PublisherZMQ shall be created");
    }

    flatbuffers::FlatBufferBuilder builder(1024);
    CreateFlatbuffersBehaviorTree(builder, root_node);

    tree_buffer_.resize(builder.GetSize());
    memcpy(tree_buffer_.data(), builder.GetBufferPointer(), builder.GetSize());

    zmq_publisher_.bind("tcp://*:1666");
    zmq_server_.bind("tcp://*:1667");

    int timeout_ms = 100;
    zmq_server_.setsockopt(ZMQ_RCVTIMEO, &timeout_ms, sizeof(int));

    active_server_ = true;

    thread_ = std::thread([this]() {
        while (active_server_)
        {
            zmq::message_t req;
            try
            {
                bool received = zmq_server_.recv(&req);
                if (received)
                {
                    zmq::message_t reply(tree_buffer_.size());
                    memcpy(reply.data(), tree_buffer_.data(), tree_buffer_.size());
                    zmq_server_.send(reply);
                }
            }
            catch (zmq::error_t& err)
            {
                std::cout << "[PublisherZMQ] just died. Exeption " << err.what() << std::endl;
                active_server_ = false;
            }
        }
    });

    createStatusBuffer();
}

PublisherZMQ::~PublisherZMQ()
{
    active_server_ = false;
    if (thread_.joinable())
    {
        thread_.join();
    }
    flush();
}

void PublisherZMQ::callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                            NodeStatus status)
{
    using namespace std::chrono;

    std::array<uint8_t, 12> transition =
        SerializeTransition(node.UID(), timestamp, prev_status, status);
    {
        std::unique_lock<std::mutex> lock(mutex_);
        transition_buffer_.push_back(transition);
    }

    if (!send_pending_)
    {
        send_pending_ = true;
        send_future_ = std::async(std::launch::async, [this]() {
            std::this_thread::sleep_for(min_time_between_msgs_);
            flush();
        });
    }
}

void PublisherZMQ::flush()
{
    zmq::message_t message;
    {
        std::unique_lock<std::mutex> lock(mutex_);

        const size_t msg_size = status_buffer_.size() + 8 + (transition_buffer_.size() * 12);

        message.rebuild(msg_size);
        uint8_t* data_ptr = static_cast<uint8_t*>(message.data());

        // first 4 bytes are the side of the header
        flatbuffers::WriteScalar<uint32_t>(data_ptr, status_buffer_.size());
        data_ptr += sizeof(uint32_t);
        // copy the header part
        memcpy(data_ptr, status_buffer_.data(), status_buffer_.size());
        data_ptr += status_buffer_.size();

        // first 4 bytes are the side of the transition buffer
        flatbuffers::WriteScalar<uint32_t>(data_ptr, transition_buffer_.size());
        data_ptr += sizeof(uint32_t);

        for (auto& transition : transition_buffer_)
        {
            memcpy(data_ptr, transition.data(), transition.size());
            data_ptr += transition.size();
        }
        transition_buffer_.clear();
        createStatusBuffer();
    }

    zmq_publisher_.send(message);
    send_pending_ = false;
    // printf("%.3f zmq send\n", std::chrono::duration<double>( std::chrono::high_resolution_clock::now().time_since_epoch() ).count());
}
}
