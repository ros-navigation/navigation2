#ifndef BT_MINITRACE_LOGGER_H
#define BT_MINITRACE_LOGGER_H

#include <cstring>
#include "abstract_logger.h"
#include "minitrace/minitrace.h"

namespace BT
{
class MinitraceLogger : public StatusChangeLogger
{
    static std::atomic<bool> ref_count;

  public:
    MinitraceLogger(TreeNode* root_node, const char* filename_json);

    virtual ~MinitraceLogger() override;

    virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                          NodeStatus status) override;

    virtual void flush() override;

  private:
    TimePoint prev_time_;
};

}   // end namespace

#endif   // BT_MINITRACE_LOGGER_H
