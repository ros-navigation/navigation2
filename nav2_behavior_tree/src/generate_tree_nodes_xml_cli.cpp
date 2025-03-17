// Copyright (c) 2024 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <vector>
#include <string>
#include <fstream>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/xml_parsing.h"

void usage(const std::string & program_name)
{
  std::cout << "TreeNodesModel description file generator for behaviortree_cpp groot tool\n"
            << "  usage: " << program_name << "[--verbose] input_file output_file\n"
            << "    input_file - line separated list of plugin libs\n"
            << "    output_file - TreeNodesModel xml file" << std::endl;
}

int main(int argc, char ** argv)
{
  BT::BehaviorTreeFactory factory;
  bool verbose = false;

  if (argc < 3) {
    usage(argv[0]);
    return 1;
  }

  int i = 1;
  if (std::string(argv[i]) == "-v" || std::string(argv[i]) == "--verbose") {
    verbose = true;
    i++;
  }
  std::string input_filename = argv[i++];
  std::string output_filename = argv[i++];

  std::vector<std::string> plugins_list;

  try {
    std::ifstream file(input_filename);
    std::string line;

    if (!file.is_open()) {
      std::cerr << "Unable to open input file: " << input_filename << std::endl;
      return 1;
    }

    while (std::getline(file, line)) {
      if (!line.empty() &&
        line[0] != '#' &&
        line.find_first_not_of(" \t\n\v\f\r") != std::string::npos)
      {
        plugins_list.push_back(line);
      }
    }
    file.close();
  } catch (const std::ios_base::failure & e) {
    std::cerr << "I/O error: " << e.what() << std::endl;
    return 1;
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  try {
    for (const auto & plugin : plugins_list) {
      if (verbose) {
        std::cout << "Loading: " << plugin << std::endl;
      }
      factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
    }
  } catch (const std::exception & e) {
    std::cerr << "Loading plugin error: " << e.what() << std::endl;
    return 1;
  }

  try {
    if (verbose) {
      std::cout << "Writing TreeNodesModel file: " << output_filename
                << "\nCompare it with the one in the git repo and update the latter if necessary.\n";
    }

    std::ofstream xml_file;
    xml_file.open(output_filename);
    xml_file << BT::writeTreeNodesModelXML(factory) << std::endl;
    xml_file.close();
  } catch (const std::ios_base::failure & e) {
    std::cerr << "I/O error: " << e.what() << std::endl;
    return 1;
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
