#include "behavior_tree_logger/bt_cout_logger.h"

namespace BT
{
std::atomic<bool> StdCoutLogger::ref_count(false);

StdCoutLogger::StdCoutLogger(TreeNode* root_node) : StatusChangeLogger(root_node)
{
    bool expected = false;
    if (!ref_count.compare_exchange_strong(expected, true))
    {
        throw std::logic_error("Only one instance of StdCoutLogger shall be created");
    }
}
StdCoutLogger::~StdCoutLogger()
{
    ref_count.store(false);
}

void StdCoutLogger::callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                             NodeStatus status)
{
    using namespace std::chrono;

    constexpr const char* whitespaces = "                         ";
    constexpr const size_t ws_count = 25;

    double since_epoch = duration<double>(timestamp).count();
    printf("[%.3f]: %s%s %s -> %s", since_epoch, node.name().c_str(),
           &whitespaces[std::min(ws_count, node.name().size())], toStr(prev_status, true),
           toStr(status, true));
    std::cout << std::endl;
}

void StdCoutLogger::flush()
{
    std::cout << std::flush;
}

}   // end namespace
