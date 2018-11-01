#ifndef BT_COUT_LOGGER_H
#define BT_COUT_LOGGER_H

#include <cstring>
#include "abstract_logger.h"

namespace BT
{
/**
 * @brief AddStdCoutLoggerToTree. Give  the root node of a tree,
 * a simple callback is subscribed to any status change of each node.
 *
 *
 * @param root_node
 * @return Important: the returned shared_ptr must not go out of scope,
 *         otherwise the logger is removed.
 */

class StdCoutLogger : public StatusChangeLogger
{
    static std::atomic<bool> ref_count;

  public:
    StdCoutLogger(TreeNode* root_node);
    ~StdCoutLogger();

    virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                          NodeStatus status) override;

    virtual void flush() override;
};

}   // end namespace

#endif   // BT_COUT_LOGGER_H
