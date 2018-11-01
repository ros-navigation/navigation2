#include "dummy_nodes.h"

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
    DummyNodes::RegisterNodes(factory);
}

namespace DummyNodes
{
BT::NodeStatus SayHello()
{
    std::cout << "Robot says: \"Hello!!!\"" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckTemperature()
{
    std::cout << "[ Temperature: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::open()
{
    _opened = true;
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::close()
{
    std::cout << "GripperInterface::close" << std::endl;
    _opened = false;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachObject::tick()
{
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

void ApproachObject::halt()
{
    setStatus(BT::NodeStatus::IDLE);
}

BT::NodeStatus SaySomething::tick()
{
    std::string msg;
    if (getParam("message", msg) == false)
    {
        // if getParam failed, use the default value
        msg = requiredNodeParameters().at("message");
    }
    std::cout << "Robot says: " << msg << std::endl;
    return BT::NodeStatus::SUCCESS;
}
}
