#include <stdio.h>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include "behavior_tree_core/bt_factory.h"

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        printf("Wrong number of arguments\nUsage: %s [filename]\n", argv[0]);
        return 1;
    }

    BT::BehaviorTreeFactory factory;

    std::set<std::string> default_nodes;
    for (auto& manifest : factory.manifests())
    {
        default_nodes.insert(manifest.registration_ID);
    }

    factory.registerFromPlugin(argv[1]);

    for (auto& manifest : factory.manifests())
    {
        if (default_nodes.count(manifest.registration_ID) > 0)
        {
            continue;
        }
        auto& params = manifest.required_parameters;
        std::cout << "---------------\n"
                  << manifest.registration_ID << " [" << manifest.type
                  << "]\n  NodeParameters: " << params.size();

        if (params.size() > 0)
        {
            std::cout << ":";
        }

        std::cout << std::endl;

        for (auto& param : params)
        {
            std::cout << "    - [Key]: \"" << param.first << "\" / [Default]: \"" << param.second
                      << "\"" << std::endl;
        }
    }

    return 0;
}
