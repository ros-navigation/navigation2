#ifndef BT_ZMQ_PUBLISHER_H
#define BT_ZMQ_PUBLISHER_H

#include <array>
#include <future>
#include <zmq.hpp>
#include "abstract_logger.h"
#include "BT_logger_generated.h"

namespace BT
{
class PublisherZMQ : public StatusChangeLogger
{
    static std::atomic<bool> ref_count;

  public:
    PublisherZMQ(TreeNode* root_node, int max_msg_per_second = 25);

    virtual ~PublisherZMQ();

  private:
    virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                          NodeStatus status) override;

    virtual void flush() override;

    TreeNode* root_node_;
    std::vector<uint8_t> tree_buffer_;
    std::vector<uint8_t> status_buffer_;
    std::vector<std::array<uint8_t, 12> > transition_buffer_;
    std::chrono::microseconds min_time_between_msgs_;

    zmq::context_t zmq_context_;
    zmq::socket_t zmq_publisher_;
    zmq::socket_t zmq_server_;

    std::atomic_bool active_server_;
    std::thread thread_;

    void createStatusBuffer();

    TimePoint deadline_;
    std::mutex mutex_;
    std::atomic_bool send_pending_;

    std::future<void> send_future_;
};
}

#endif   // BT_ZMQ_PUBLISHER_H
