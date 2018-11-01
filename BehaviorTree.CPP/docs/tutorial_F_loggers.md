
__BehaviorTree.CPP__ provides an extensible set of Loggers, i.e. 
a mechanism to record and/or display all the state transitions in out tree.


## Example

We can attach multiple loggers to a tree.

To do this, we pass the root of the tree. 

```c++ hl_lines="21 22 23 25"
#include "behavior_tree_core/xml_parsing.h"
#include "behavior_tree_logger/bt_cout_logger.h"
#include "behavior_tree_logger/bt_minitrace_logger.h"
#include "behavior_tree_logger/bt_file_logger.h"

#ifdef ZMQ_FOUND
#include "behavior_tree_logger/bt_zmq_publisher.h"
#endif

using namespace BT;

int main()
{
    BT::BehaviorTreeFactory factory;
    // Load and register all the Custom TreeNodes from plugin
	factory.registerFromPlugin("./libcrossdoor_nodes.so");

    auto tree = buildTreeFromFile(factory, "crossdoor.xml");

    // Create some loggers.
    StdCoutLogger   logger_cout(tree.root_node);
    FileLogger      logger_file(tree.root_node, "bt_trace.fbl");
    MinitraceLogger logger_minitrace(tree.root_node, "bt_trace.json");
#ifdef ZMQ_FOUND
    PublisherZMQ    publisher_zmq(tree.root_node);
#endif
    
    // Keep on ticking until you get either a SUCCESS or FAILURE 
    NodeStatus status = NodeStatus::RUNNING;
    while( status == NodeStatus::RUNNING )
    {
        status = tree.root_node->executeTick();
        CrossDoor::SleepMS(1); // optional sleep to avoid "busy loops"
    }
    return 0;
}

```

## StdCoutLogger

It simply prints the state transition on __std::cout__.


## FileLogger

It stores the state transitions (and their timestamp) in the binary file
called "bt_trace.fbl".

To visualize the content of this file, use the command line tool 
__bt_log_cat__.

## MinitraceLogger

This logger stores the states trnasitions and durations in a JSON file format. 

Its goal is to show the time required by a TreeNode to complete its tick() operation.

This tracing format can be visualized in the __Chrome__ tracer viewer; you can access it
typing this in the address bar: __chrome://tracing__.

For more information, refer to: [MiniTrace (GitHub)](https://github.com/hrydgard/minitrace)

## PublisherZMQ

It publishes state transitions in real-time using [ZMQ](http://zeromq.org/).

You can record them using the command line tool __bt_recorder__.








