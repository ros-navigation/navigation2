# Dynamic Params

The nav2_dynamic_params package implements a class for the validation of dynamic ROS2 parameters as well as a class which enables tracking and convenient access to dynamic parameters. This package was motivated by the lack of 
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
- adds parameters from any node to be tracked
- registers internal callback to filter events and pass to user-defined callback
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
dynamic_params_client = new nav2_dynamic_params::DynamicParamsClient(node);

// Add parameters by node. Note that there are different ways to add parameters
// The namespace must be provided, if applicable
dynamic_params_client->add_parameters("example_node_A", {"foo"});

// Add all existing parameter on node. If node is not available for service,
// then none of its parameters will be registered
dynamic_params_client->add_parameters_on_node("example_node_B");

// If a parameter is specified but not currently set or node is unavailable,
// it will be registered as PARAMETER_NOT_SET.
dynamic_params_client->add_parameters("some_namespace", "example_node_C", {"baz", "bar"});

// without node path, adding only parameters will grab parameters from member node
dynamic_params_client->add_parameters({"foobar", "foobaz"});

// Create a callback for parameter events
std::function<void()> callback = [this]() -> void
  {
    // Check if a parameter is part of the latest event
    if (dynamic_params_client->is_in_event("example_node_B", "bar")) {
      RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params_client"),
        "'example_node_B/bar' is in this event!");
    }

    double foo;
    dynamic_params_client->get_event_param("example_node_A", "foo", foo);

    int bar_B;
    dynamic_params_client->get_event_param_or("example_node_B", "bar", bar_B, 2);

    int bar_C;
    dynamic_params_client->get_event_param_or("some_namespace/example_node_C", "bar", bar_C, 3);

    std::string baz;
    dynamic_params_client->get_event_param_or("some_namespace", "example_node_C",
      "baz", baz, std::string("default"));

    // Parameter not set on node
    double foobar;
    dynamic_params_client->get_event_param_or("foobar", foobar, 5.5);

    int foobaz;
    dynamic_params_client->get_event_param_or("foobaz", foobaz, 25);
  };
  
// Set user callback in DynamicParamsClient, to be invoked if a tracked parameter is found in the incoming event
// By default, the callback will be invoked immediately with an empty event
dynamic_params_client->set_callback(callback);
```

Here, the DynamicParamsClient is created to listen for parameter events from several nodes. When parameters are added by namespace and node name, the DynamicParamsClient initializes the current values off the nodes, creates a subscription to that namespace's parameter events topic, and registers an internal callback. If nodes are unavailable or a parameter is not yet set, the parameters are still registered in the cached parameter map as PARAMETER_NOT_SET. The utility of the map of cached parameters is to faciliate access to these dynamic parameters at any time, where even if the parameter of interest is not part of the latest event, one may receive its current value or use a provided default if unavailable. The user-defined callback is applied whenever an incoming parameter event matches a parameter currently stored in the cached map. With parameter event messages now containing a fully qualified path to the host node of the event, duplicate parameters names may be tracked across different nodes regardless of namespace.   

## Future Plans / TODO
- Validate parameters set at launch. Currently, launched parameters are set on the node before the validation callback is created
- Set validation types and bounds at launch time via file (within YAML?)
- Provide GUI for dynamic parameters
