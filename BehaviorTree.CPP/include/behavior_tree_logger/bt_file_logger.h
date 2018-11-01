#ifndef BT_FILE_LOGGER_H
#define BT_FILE_LOGGER_H

#include <fstream>
#include <deque>
#include <array>
#include "abstract_logger.h"

namespace BT
{
class FileLogger : public StatusChangeLogger
{
  public:
    FileLogger(TreeNode* root_node, const char* filename, uint16_t buffer_size = 10);

    virtual ~FileLogger() override;

    virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                          NodeStatus status) override;

    virtual void flush() override;

  private:
    std::ofstream file_os_;

    std::chrono::high_resolution_clock::time_point start_time;

    std::vector<std::array<uint8_t, 12> > buffer_;

    bool buffer_max_size_;
};

}   // end namespace

#endif   // BT_FILE_LOGGER_H
