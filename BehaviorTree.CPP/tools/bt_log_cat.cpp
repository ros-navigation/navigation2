#include <stdio.h>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include "behavior_tree_logger/BT_logger_generated.h"

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        printf("Wrong number of arguments\nUsage: %s [filename]\n", argv[0]);
        return 1;
    }

    FILE* file = fopen(argv[1], "rb");

    if (!file)
    {
        printf("Failed to open file: [%s]\n", argv[1]);
        return 1;
    }

    fseek(file, 0L, SEEK_END);
    const size_t length = ftell(file);
    fseek(file, 0L, SEEK_SET);
    std::vector<char> buffer(length);
    fread(buffer.data(), sizeof(char), length, file);
    fclose(file);

    const int bt_header_size = flatbuffers::ReadScalar<uint32_t>(&buffer[0]);

    auto behavior_tree = BT_Serialization::GetBehaviorTree(&buffer[4]);

    std::unordered_map<uint16_t, std::string> names_by_uid;
    std::unordered_map<uint16_t, const BT_Serialization::TreeNode*> node_by_uid;

    for (const BT_Serialization::TreeNode* node : *(behavior_tree->nodes()))
    {
        names_by_uid.insert({node->uid(), std::string(node->instance_name()->c_str())});
        node_by_uid.insert({node->uid(), node});
    }

    printf("----------------------------\n");

    std::function<void(uint16_t, int)> recursiveStep;

    recursiveStep = [&](uint16_t uid, int indent) {
        for (int i = 0; i < indent; i++)
        {
            printf("    ");
            names_by_uid[uid] = std::string("   ") + names_by_uid[uid];
        }
        printf("%s\n", names_by_uid[uid].c_str());
        std::cout << std::flush;

        const auto& node = node_by_uid[uid];

        for (size_t i = 0; i < node->children_uid()->size(); i++)
        {
            recursiveStep(node->children_uid()->Get(i), indent + 1);
        }
    };

    recursiveStep(behavior_tree->root_uid(), 0);

    printf("----------------------------\n");

    constexpr const char* whitespaces = "                         ";
    constexpr const size_t ws_count = 25;

    auto printStatus = [](BT_Serialization::Status status) {
        switch (status)
        {
            case BT_Serialization::Status::SUCCESS:
                return ("\x1b[32m"
                        "SUCCESS"
                        "\x1b[0m");   // RED
            case BT_Serialization::Status::FAILURE:
                return ("\x1b[31m"
                        "FAILURE"
                        "\x1b[0m");   // GREEN
            case BT_Serialization::Status::RUNNING:
                return ("\x1b[33m"
                        "RUNNING"
                        "\x1b[0m");   // YELLOW
            case BT_Serialization::Status::IDLE:
                return ("\x1b[36m"
                        "IDLE   "
                        "\x1b[0m");   // CYAN
        }
        return "Undefined";
    };

    for (size_t index = bt_header_size + 4; index < length; index += 12)
    {
        const uint16_t uid = flatbuffers::ReadScalar<uint16_t>(&buffer[index + 8]);
        const std::string& name = names_by_uid[uid];
        const uint32_t t_sec = flatbuffers::ReadScalar<uint32_t>(&buffer[index]);
        const uint32_t t_usec = flatbuffers::ReadScalar<uint32_t>(&buffer[index + 4]);

        printf("[%d.%06d]: %s%s %s -> %s\n", t_sec, t_usec, name.c_str(),
               &whitespaces[std::min(ws_count, name.size())],
               printStatus(flatbuffers::ReadScalar<BT_Serialization::Status>(&buffer[index + 10])),
               printStatus(flatbuffers::ReadScalar<BT_Serialization::Status>(&buffer[index + 11])));
    }

    return 0;
}
