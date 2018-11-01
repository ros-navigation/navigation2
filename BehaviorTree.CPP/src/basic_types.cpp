#include "behavior_tree_core/basic_types.h"
#include <cstring>

namespace BT
{
const char* toStr(const NodeStatus& status, bool colored)
{
    if (!colored)
    {
        switch (status)
        {
            case NodeStatus::SUCCESS:
                return "SUCCESS";
            case NodeStatus::FAILURE:
                return "FAILURE";
            case NodeStatus::RUNNING:
                return "RUNNING";
            case NodeStatus::IDLE:
                return "IDLE";
        }
    }
    else
    {
        switch (status)
        {
            case NodeStatus::SUCCESS:
                return ("\x1b[32m"
                        "SUCCESS"
                        "\x1b[0m");   // RED
            case NodeStatus::FAILURE:
                return ("\x1b[31m"
                        "FAILURE"
                        "\x1b[0m");   // GREEN
            case NodeStatus::RUNNING:
                return ("\x1b[33m"
                        "RUNNING"
                        "\x1b[0m");   // YELLOW
            case NodeStatus::IDLE:
                return ("\x1b[36m"
                        "IDLE"
                        "\x1b[0m");   // CYAN
        }
    }
    return "Undefined";
}

const char* toStr(const NodeType& type)
{
    switch (type)
    {
        case NodeType::ACTION:
            return "Action";
        case NodeType::CONDITION:
            return "Condition";
        case NodeType::DECORATOR:
            return "Decorator";
        case NodeType::CONTROL:
            return "Control";
        case NodeType::SUBTREE:
            return "SubTree";
        default:
            return "Undefined";
    }
}

template <>
std::string convertFromString<std::string>(const std::string& str)
{
    return str;
}

template <>
const char* convertFromString<const char*>(const std::string& str)
{
    return str.c_str();
}

template <>
int convertFromString<int>(const std::string& str)
{
    return std::stoi(str.c_str());
}

template <>
unsigned convertFromString<unsigned>(const std::string& str)
{
    return std::stoul(str.c_str());
}

template <>
double convertFromString<double>(const std::string& str)
{
    return std::stod(str);
}

template <>
bool convertFromString<bool>(const std::string& str)
{
    if (str.size() == 1)
    {
        if (str[0] == '0')
        {
            return false;
        }
        else if (str[0] == '1')
        {
            return true;
        }
        else
        {
            std::runtime_error("invalid bool conversion");
        }
    }
    else if (str.size() == 4)
    {
        if (str == "true" || str == "TRUE" || str == "True")
        {
            return true;
        }
        else
        {
            std::runtime_error("invalid bool conversion");
        }
    }
    else if (str.size() == 5)
    {
        if (str == "false" || str == "FALSE" || str == "False")
        {
            return false;
        }
        else
        {
            std::runtime_error("invalid bool conversion");
        }
    }

    std::runtime_error("invalid bool conversion");
    return false;
}

template <>
NodeStatus convertFromString<NodeStatus>(const std::string& str)
{
    for (auto status :
         {NodeStatus::IDLE, NodeStatus::RUNNING, NodeStatus::SUCCESS, NodeStatus::FAILURE})
    {
        if (std::strcmp(toStr(status), str.c_str()) == 0)
        {
            return status;
        }
    }
    throw std::invalid_argument(std::string("Cannot convert this to NodeStatus: ") + str);
}

template <>
NodeType convertFromString<NodeType>(const std::string& str)
{
    for (auto status : {NodeType::ACTION, NodeType::CONDITION, NodeType::CONTROL,
                        NodeType::DECORATOR, NodeType::SUBTREE, NodeType::UNDEFINED})
    {
        if (std::strcmp(toStr(status), str.c_str()) == 0)
        {
            return status;
        }
    }
    throw std::invalid_argument(std::string("Cannot convert this to NodeType: ") + str);
}

std::ostream& operator<<(std::ostream& os, const NodeType& type)
{
    os << toStr(type);
    return os;
}

std::ostream& operator<<(std::ostream& os, const NodeStatus& status)
{
    os << toStr(status);
    return os;
}

std::vector<std::string> splitString(const std::string& strToSplit, char delimeter)
{
    std::stringstream ss(strToSplit);
    std::string item;
    std::vector<std::string> splitted_strings;
    while (std::getline(ss, item, delimeter))
    {
        splitted_strings.push_back(item);
    }
    return splitted_strings;
}

}   // end namespace
