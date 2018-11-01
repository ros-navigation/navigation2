# How to use NodeParameters

NodeParameters are like arguments passed to a function.

They are a map of __key/value__ pairs (both strings) that are usually
parsed from file.

To create a TreeNodes that accepts NodeParameters, you must follow these rules:

- Use inheritance. NodeParameters are __not supported__ by *SimpleActionNodes* nor
 *SimpleConditionNodes*.

- You must provide a constructor with the following signature:

``` c++
MyAction(const std::string& name, const BT::NodeParameters& params) 
```

- The following static member function must be defined:

``` c++
static const BT::NodeParameters& requiredNodeParameters()
```


## Example: an Action requiring the parameter "message"

`SaySomething` is a simple synchronous ActionNodeBase which will print the 
string passed in the NodeParameter called "message".

Please note:

- The constructor signature.

- The __static__ method `requiredNodeParameters()` contains a single key/value pair.
  The string "default message" is the default value.
  
- Parameters must be accessed using the method `getParam()`, preferably inside the
`tick()` method.

``` c++ hl_lines="5 9 18"
class SaySomething: public ActionNodeBase
{
public:
    // There must be a constructor with this signature
    SaySomething(const std::string& name, const NodeParameters& params):
        ActionNodeBase(name, params) {}

    // It is mandatory to define this static method.
    static const NodeParameters& requiredNodeParameters()
    {
        static NodeParameters params = {{"message","default message"}};
        return params;
    }

    virtual NodeStatus tick() override
    {
		std::string msg;
		if( getParam("message", msg) == false )
		{
			// if getParam failed, use the default value
			msg = requiredNodeParameters().at("message");
		}
		std::cout << "Robot says: " << msg << std::endl;
		return BT::NodeStatus::SUCCESS;
	}
    virtual void halt() override {}
};
```


## Example: conversion to user defined C++ types

In the next example we have a user defined type `Pose2D`.

``` c++
struct Pose2D
{
    double x,y,theta;
};
```

If we want the method `getParam()` to be able to parse a string
and store its value into a `Pose2D`, we must provide our own template specialization
of `convertFromString<T>()`.

In this case, the text representation of a `Pose2D` is three real numbers separated by 
semicolons, like this:

    "1.1;-2.32;0.4"

Since this is a common pattern, the library provide the helper function `BT::splitString`.


``` c++ hl_lines="6"
// use this namespace
namespace BT{

// This template specialization is needed if you want
// to AUTOMATICALLY convert a NodeParameter into a Pose2D
template <> Pose2D BT::convertFromString(const std::string& key)
{
    // Three real numbers separated by semicolons.
    // You may use <boost/algorithm/string/split.hpp>  if you prefer
    auto parts = BT::splitString(key, ';');
    if( parts.size() != 3)
    {
        throw std::runtime_error("invalid input)");
    }
    else{
        Pose2D output;
        output.x     = convertFromString<double>( parts[0] );
        output.y     = convertFromString<double>( parts[1] );
        output.theta = convertFromString<double>( parts[2] );
        return output;
    }
}

} // end namespace
```

We now define a __asynchronous__ ActionNode called __MoveBaseAction__.

The method tick() of an `ActionNode` is executed in its own thread.

The method `getParam()` will call the function `convertFromString<Pose2D>()` under the hood;
alternatively, we can use the latter funtion directly, for instance to convert the default
string "0;0;0" into a Pose2D.

``` c++ hl_lines="20 21 22 23 24"
// This is an asynchronous operation that will run in a separate thread.
// It requires the NodeParameter "goal". 
// If the key is not provided, the default value "0;0;0" is used instead.
class MoveBaseAction: public ActionNode
{
public:

    MoveBaseAction(const std::string& name, const NodeParameters& params):
        ActionNode(name, params) {}

    static const BT::NodeParameters& requiredNodeParameters()
    {
        static BT::NodeParameters params = {{"goal","0;0;0"}};
        return params;
    }

	virtual NodeStatus tick() override
	{
	    Pose2D goal;
        if( getParam<Pose2D>("goal", goal) == false )
        {
            auto default_goal =  requiredNodeParameters().at("goal");
            goal = BT::convertFromString<Pose2D>( default_goal_value );
        }
        
		printf("[ MoveBase: STARTED ]. goal: x=%.f y=%.1f theta=%.2f\n",
			   goal.x, goal.y, goal.theta);

		halt_requested_ = false;
		
		int count = 0;
		// "compute" for 250 milliseconds or until halt_requested_
		while( !halt_requested_ && count++ < 25)
		{
			SleepMilliseconds(10);
		}

		std::cout << "[ MoveBase: FINISHED ]" << std::endl;
		return halt_requested_ ? NodeStatus::FAILURE : 
		                         NodeStatus::SUCCESS;
	}

	virtual void halt() override 
	{
		halt_requested_ = true;
	}
private:
    bool halt_requested_;
};

```

## NodeParameters in the XML

To pass the parameter from a XML, use attributes:

``` XML
<Sequence>
	<MoveBaseAction goal="41.2;13.5;0.7"/>
	<SaySomething   message="Destination reached"/>
	<SaySomething/> <!-- No parameter passed  --> 
</Sequence>	
```

Expected output:
    
    
    [ MoveBase: STARTED ]: goal: x=41.2 y=13.5 theta=0.7
    [ MoveBase: FINISHED ]
    Robot says: Destination reached
    Robot says: default message





