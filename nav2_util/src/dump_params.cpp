// Copyright (c) 2019 Intel Corporation
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
// limitations under the License.

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "boost/program_options.hpp"
#include "boost/tokenizer.hpp"
#include "boost/foreach.hpp"
#include "boost/algorithm/algorithm.hpp"
#include "boost/algorithm/string/split.hpp"
#include "boost/algorithm/string/classification.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rclcpp/rclcpp.hpp"

namespace po = boost::program_options;
namespace alg = boost::algorithm;

using namespace std::chrono_literals;

static std::vector<std::string>
get_param_names_for_node(rclcpp::Node::SharedPtr node, std::string node_name)
{
  auto client = node->create_client<rcl_interfaces::srv::ListParameters>(
    node_name + "/list_parameters");

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("client interrupted while waiting for service to appear.");
    }

    throw std::runtime_error(
            std::string("ListParameters service for ") +
            node_name + " not available");
  }

  auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
  auto result_future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result_future, 1s) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error(std::string("service call to \"") + node_name + "\" failed");
  }

  return result_future.get()->result.names;
}

static std::vector<rcl_interfaces::msg::ParameterValue>
get_param_values_for_node(
  rclcpp::Node::SharedPtr node, std::string node_name,
  std::vector<std::string> & param_names)
{
  auto client = node->create_client<rcl_interfaces::srv::GetParameters>(
    node_name + "/get_parameters");

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("client interrupted while waiting for service to appear.");
    }

    throw std::runtime_error(
            std::string("GetParameters service for ") +
            node_name + " not available");
  }

  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = param_names;

  auto result_future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result_future, 1s) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error(std::string("service call to \"") + node_name + "\" failed");
  }

  return result_future.get()->values;
}

static std::vector<rcl_interfaces::msg::ParameterDescriptor>
get_param_descriptors_for_node(
  rclcpp::Node::SharedPtr node, std::string node_name,
  std::vector<std::string> & param_names)
{
  auto client = node->create_client<rcl_interfaces::srv::DescribeParameters>(
    node_name + "/describe_parameters");

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("client interrupted while waiting for service to appear.");
    }

    throw std::runtime_error(
            std::string("DescribeParameters service for ") +
            node_name + " not available");
  }

  auto request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();
  request->names = param_names;

  auto result_future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result_future, 1s) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error(std::string("service call to \"") + node_name + "\" failed");
  }

  return result_future.get()->descriptors;
}

// A local version to avoid trailing zeros
template<typename T>
static std::string to_string(const T a_value)
{
  std::ostringstream out;
  out << a_value;
  return out.str();
}

static void
print_yaml(
  const std::string node_name, std::vector<std::string> & param_names,
  const std::vector<rcl_interfaces::msg::ParameterValue> & param_values,
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & param_descriptors, bool verbose)
{
  std::cout << node_name << ":" << std::endl;
  std::cout << "  ros__parameters:" << std::endl;

  for (unsigned i = 0; i < param_names.size(); i++) {
    std::function<std::string(void)> pf;

    switch (param_values[i].type) {
      case rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET:
        pf = []() {return "NOT_SET";};
        break;

      case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
        pf = [param_values, i]() {return param_values[i].bool_value ? "True" : "False";};
        break;

      case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
        pf = [param_values, i]() {return std::to_string(param_values[i].integer_value);};
        break;

      case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
        pf = [param_values, i]() {return to_string(param_values[i].double_value);};
        break;

      case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
        pf = [param_values, i]() {
            return std::string("\"") + param_values[i].string_value + std::string("\"");
          };
        break;

      case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
      case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
      case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
      case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
      case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
        pf = [param_values, i]() {
            std::stringstream stream;
            stream << "[";
            auto num_items = param_values[i].string_array_value.size();
            for (unsigned j = 0; j < num_items; j++) {
              switch (param_values[i].type) {
                case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
                  stream << param_values[i].byte_array_value[j];
                  break;
                case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
                  stream << (param_values[i].bool_array_value[j] ? "True" : "False");
                  break;
                case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
                  stream << param_values[i].integer_array_value[j];
                  break;
                case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
                  stream << param_values[i].double_array_value[j];
                  break;
                case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
                  stream << "\"" << param_values[i].string_array_value[j] << "\"";
                  break;
              }

              if (j != num_items - 1) {
                stream << ", ";
              }
            }
            stream << "]";
            return stream.str();
          };
        break;
    }

    // Use a field width wide enough for all of the headers
    auto fw = 30;

    if (verbose) {
      std::cout << "    " << std::left << std::setw(fw + 2) << param_names[i] + ":" << pf() << "\n";
    } else {
      std::cout << "    " << param_names[i] << ": " << pf() << "\n";
    }

    if (verbose) {
      std::cout << "    # " << std::left << std::setw(fw) << "Range: ";
      if (param_descriptors[i].floating_point_range.size()) {
        auto range = param_descriptors[i].floating_point_range[0];
        std::cout << range.from_value << ";" <<
          range.to_value << ";" <<
          range.step << "\n";
      } else if (param_descriptors[i].integer_range.size()) {
        auto range = param_descriptors[i].integer_range[0];
        std::cout << range.from_value << ";" <<
          range.to_value << ";" <<
          range.step << "\n";
      } else {
        std::cout << "N/A\n";
      }

      std::cout << "    # " << std::left << std::setw(fw) << "Description: " <<
        param_descriptors[i].description << "\n";
      std::cout << "    # " << std::left << std::setw(fw) << "Additional constraints: " <<
        param_descriptors[i].additional_constraints << "\n";
      std::cout << "    # " << std::left << std::setw(fw) << "Read-only: " <<
      (param_descriptors[i].read_only ? "True" : "False") << "\n";

      std::cout << std::endl;
    }
  }

  std::cout << std::endl;
}

static void
print_markdown(
  const std::string node_name, std::vector<std::string> & param_names,
  const std::vector<rcl_interfaces::msg::ParameterValue> & param_values,
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & param_descriptors,
  bool verbose)
{
  std::cout << "## " << node_name << " Parameters" << "\n";

  if (verbose) {
    std::cout << "|Parameter|Default Value|Range|Description|Additional Constraints|Read-Only|" <<
      "\n";
    std::cout << "|---|---|---|---|---|---|" << "\n";
  } else {
    std::cout << "|Parameter|Default Value|" << "\n";
    std::cout << "|---|---|" << "\n";
  }

  for (unsigned i = 0; i < param_names.size(); i++) {
    std::function<std::string(void)> pf;

    switch (param_values[i].type) {
      case rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET:
        pf = []() {return "NOT_SET";};
        break;

      case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
        pf = [param_values, i]() {return param_values[i].bool_value ? "True" : "False";};
        break;

      case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
        pf = [param_values, i]() {return std::to_string(param_values[i].integer_value);};
        break;

      case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
        pf = [param_values, i]() {return to_string(param_values[i].double_value);};
        break;

      case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
        pf = [param_values, i]() {
            return std::string("\"") + param_values[i].string_value + std::string("\"");
          };
        break;

      case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
      case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
      case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
      case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
      case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
        pf = [param_values, i]() {
            std::stringstream stream;
            stream << "[";
            auto num_items = param_values[i].string_array_value.size();
            for (unsigned j = 0; j < num_items; j++) {
              switch (param_values[i].type) {
                case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
                  stream << param_values[i].byte_array_value[j];
                  break;
                case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
                  stream << (param_values[i].bool_array_value[j] ? "True" : "False");
                  break;
                case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
                  stream << param_values[i].integer_array_value[j];
                  break;
                case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
                  stream << param_values[i].double_array_value[j];
                  break;
                case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
                  stream << "\"" << param_values[i].string_array_value[j] << "\"";
                  break;
              }

              if (j != num_items - 1) {
                stream << ", ";
              }
            }
            stream << "]";
            return stream.str();
          };
        break;
    }

    std::cout << "|" << param_names[i] << "|" << pf();

    if (verbose) {
      if (param_descriptors[i].floating_point_range.size()) {
        auto range = param_descriptors[i].floating_point_range[0];
        std::cout << "|" <<
          range.from_value << ";" <<
          range.to_value << ";" <<
          range.step << "|";
      } else if (param_descriptors[i].integer_range.size()) {
        auto range = param_descriptors[i].integer_range[0];
        std::cout << "|" <<
          range.from_value << ";" <<
          range.to_value << ";" <<
          range.step << "|";
      } else {
        // No range specified
        std::cout << "|N/A";
      }

      std::cout << "|" <<
        param_descriptors[i].description << "|" <<
        param_descriptors[i].additional_constraints << "|" <<
      (param_descriptors[i].read_only ? "True" : "False");
    }

    // End the parameter
    std::cout << "|\n";
  }

  std::cout << std::endl;
}

template<typename T>
struct option_sequence
{
  std::vector<T> values;
};

template<typename T>
void
validate(
  boost::any & v, const std::vector<std::string> & values,
  option_sequence<T> * /*target_type*/, int)
{
  std::vector<T> result;
  typedef std::vector<std::string> strings;
  for (strings::const_iterator iter = values.begin(); iter != values.end(); ++iter) {
    strings tks;
    alg::split(tks, *iter, alg::is_any_of(","));
    for (strings::const_iterator tk = tks.begin(); tk != tks.end(); ++tk) {
      result.push_back(boost::lexical_cast<T>(*tk));
    }
  }
  v = option_sequence<T>();
  boost::any_cast<option_sequence<T> &>(v).values.swap(result);
}

#ifdef _WIN32
static const char * basename(const char * filepath)
{
  const char * base = std::strrchr(filepath, '/');
  return base ? (base + 1) : filepath;
}
#endif

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto dump_params_node = rclcpp::Node::make_shared("dump_params");

  try {
    po::options_description desc("Options");

    /* *INDENT-OFF* */
    desc.add_options()("help,h", "Print help message")
      ("node_names,n", po::value<option_sequence<std::string>>(),
        "A list of comma-separated node names")
      ("format,f", po::value<std::string>(), "The format to dump ('yaml' or 'markdown')")
      ("verbose,v", "Verbose option")
    ;
    /* *INDENT-ON* */

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << "Usage: " << basename(argv[0]) << "\n";
      std::cout << desc << "\n";
      return 1;
    }

    std::vector<std::string> node_names;

    if (vm.count("node_names")) {
      node_names = vm["node_names"].as<option_sequence<std::string>>().values;
    } else {
      node_names = dump_params_node->get_node_names();
    }

    bool verbose = vm.count("verbose");

    for (std::string target_node_name : node_names) {
      // Skip hidden nodes
      if (target_node_name[1] == '_') {
        continue;
      }

      try {
        auto param_names = get_param_names_for_node(dump_params_node, target_node_name);
        auto param_values =
          get_param_values_for_node(dump_params_node, target_node_name, param_names);
        auto param_descriptors = get_param_descriptors_for_node(
          dump_params_node, target_node_name,
          param_names);

        if (!vm.count("format")) {
          // Default to YAML if the format hasn't been specified
          print_yaml(target_node_name, param_names, param_values, param_descriptors, verbose);
        } else {
          auto format = vm["format"].as<std::string>();
          if (format == "md" || format == "markdown") {
            print_markdown(target_node_name, param_names, param_values, param_descriptors, verbose);
          } else {
            if (format != "yaml") {
              std::cerr << "Unknown output format specified, defaulting to 'yaml'" << std::endl;
            }
            print_yaml(target_node_name, param_names, param_values, param_descriptors, verbose);
          }
        }
      } catch (std::exception & e) {
        std::cerr << "Error: " << e.what() << "\n" << std::endl;
      }
    }
  } catch (po::error & e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
