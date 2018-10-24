# Dynamic Params

The nav2_dynamic_params package implements a class for the validation of dynamic ROS2 parameters as well as a class which wraps
ROS2 parameter clients for convenient access to dynamic parameters. This package was motivated by the lack of 
[DynamicReconfigure](https://github.com/ros/dynamic_reconfigure) in ROS2, which while not ported, remains an important tool for the 
handling of dynamic parameters during run-time. Thus, using current ROS2-supported parameter features, this package aims to fill 
the gap in functionality. 

## ROS2 Parameters

In ROS2, rather than on a parameter server, parameters are hosted on the nodes themselves. Nodes may get and set their own
parameters via the node interface or may create a parameter client to any remote node hosting parameters. By default, any client to
a node may get and set parameters to that node. A node, however, may register a callback for parameter change requests and choose
to accept or deny such requests based on arbitrary criteria. In this vain, nodes are responsible for the validation of their own
parameters. Parameter clients, on the other hand, may register a callback to parameter events from a remote node. Thus, all
parameters could be considered by default to be dynamic in ROS2 as anyone can get or set them and receive callbacks. 

## Dynamic Params Overview

Two classes are implemented to support and extend the functionality of ROS2 parameters: DynamicParamsValidator and
DynamicParamsClient

### DynamicParamsValidator
- validates parameters against type (using rclcpp::ParameterType)
- validates parameters against specified upper/lower bounds
- allows for providing custom user validation callback 
- option to add parameters as static and reject changes
- optional to reject all new parameters

### DynamicParamsClient
- creates internal parameter clients to remote nodes
- registers callback to all internal clients
- keeps cached map of dynamic parameter changes
- provides interface function calls to access latest parameter change or default value within callback

## Example Code
Below is an example of using the DynamicParamsValidator:
```C++

auto node = rclcpp::Node::make_shared("example_dynamic_params_validator");

// Create DynamicParamsValidator given node, rejecting new parameters
auto param_validator = new nav2_dynamic_params::DynamicParamsValidator(node, true);

// Add Parameters to Validator
param_validator->add_param("foo", rclcpp::ParameterType::PARAMETER_DOUBLE);
param_validator->add_param("bar", rclcpp::ParameterType::PARAMETER_INTEGER, {0, 10});

// Set Parameters on node
node->set_parameters({rclcpp::Parameter("foo", 2.0), rclcpp::Parameter("bar", 3)});

// Make "foo" a static parameter
param_validator->add_static_params({"foo"});
```
In the code sample above, the node will be able to validate parameters "foo" and "bar" from requests that violate their
respective type and upper and lower bounds, whilst rejecting new parameters to be added. If a paremeter has already been set,
either within code or from launch, setting the parameter as static prevents future changes. 

For clients of parameters that exist on nodes, whether themselves, or remote, they can use the DynamicParamsClient as below:

```C++

auto node = rclcpp::Node::make_shared("example_dynamic_params_client");

// Create DynamicReconfigureClient
dynamic_params_client = new nav2_dynamic_params::DynamicParamsClient(node, 
  {"example_dynamic_params_validator", "other_node"});

// Create a callback for parameter events
std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> callback = [node](
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
  {
    double foo;
    // Grabs the value of "foo" from the event or from cache internal to DynamicParamsClient   
    dynamic_params_client->get_event_param(event, "foo", foo);
    
    int bar;
    // If parameter "bar" doesn't exist yet or has not been set, a default value may be  used
    dynamic_params_client->get_event_param(event, "bar", bar, 4);
    
    double foobar;
    // Parameter "foobar" exists on "other_node"
    dynamic_params_client->get_event_param(event, "foobar", foobar, 5.5);
  };
// Pass callback to DynamicParamsClient, to be added to parameter clients for all nodes
dynamic_params_client->set_callback(callback);
```

Here, the DynamicParamsClient is created to listen to events for two nodes, "example_dynamic_params_validator" and some
"other_node". Upon construction, the object *currently* creates a vector of internal parameter clients for each node and adds all
current parameters on the node to its map cache of parameters. The utility of the cached parameters is to faciliate access within
callbacks, where even if the parameter of interest is not part of the event, one may receive its current value. The user-defined
callback is applied to all the parameter clients for each node, however, even if only one node triggers an event, only its 
respective client will receive the callback and access its values accordingly. 

## Future Redesign
One drawback of the aforementioned design above is that user-defined callbacks for parameter events will be triggered even if the
the event does not contain a dynamic parameter of interest (for example some other node changes a parameter that is irrelevant to
your node and callback). In the current model, the callback would be called but none of the parameters of interest would change.
This may be undesirable behavior, especially if there is some expense to calling the callback. Thus, to prevent this, another
proposal is to register an internal callback to filter the events, and only pass the event to the user-defined callback if it
contains a desired dynamic parameter. This chain of callbacks would prevent unnecessary calls to the user-callback.   
Another drawback is that the current design does not support duplicate parameter names. For example, if two nodes hosted the
parameter "baz", then we would not be able to resolve the two. A proposed design is to specify by node the parameters of interest,
internally created a mapping between nodes and dynamic parameters. This mapping can be used to resolve different node parameters
which may have duplicate parameter names.

### Example Proposed Interface (example proposed by @mjeronimo)

```C++
// Proposed programming model for DynamicParamsClient:

auto dynamic_params_client = new nav2_dynamic_params::DynamicParamsClient(node);

// This automatically adds (listens to changes for) all parameters for Node "A"
dynamic_params_client->add_params("A");
// Register interest in only param4 and param5 from node B
dynamic_params_client->add_params("B", {"param4", "param5"});
// Register interest in param4 and param7 from node C. Notice that parameter names
// don't have to be unique system-wide
dynamic_params_client->add_params("C", {"param4", "param7"});

// The user's callback will be called with the set of parameters it is interested in:
// All parameters from A; param4 and param5 from node B; and param4 and param7 from Node C
// This is a logical group of parameters
dynamic_params_client->set_callback(user_callback);

// Can potentially add more nodes after setting the callback
dynamic_params_client->add_params("D", { "param1", "param2" });
```
## Future Plans / TODO
- Validate parameters set at launch. Currently, launched parameters are set on the node before the validation callback is created
- Set validation types and bounds at launch time via file (within YAML?)
- Trigger DynamicParamsClient callback upon initialization. 
- Provide GUI for dynamic parameters





