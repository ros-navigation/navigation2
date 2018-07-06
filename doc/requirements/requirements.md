# ROS 2 Navigation System Requirements

The ROS 2 Navigation System ("Navigation System") is the control system that enables a robot to autonomously reach a goal state, such as a specific position and orientation relative to a given map. Provided with a current pose, a map, and a goal, such as a destination pose, the Navigation System generates a plan to reach the goal, and outputs control commands to autonomously drive the robot, respecting any safety constraints and avoiding obstacles encountered along the way. 

This document lists the requirements for the ROS 2 Navigation System. The ROS 2 Navigation System will be a generalization of the ROS 1 navigation stack and will address some of its known limitations. 

# 1. Introduction

This section describes the format of each requirement, the keywords used in the requirements, and the basic concepts needed to define and understand the requirements.

## 1.1 Requirement Fields

Each requirement is presented in tabular form with the following fields:

* **Id** - A unique identifier for the requirement
* **Handle** - A short, scoped, description summarizing the essence of the requirement
* **Priority** - An associated priority level: **1** (high), **2** (medium), and **3** (low)
* **Requirement** - The requirement itself, stated in clear, concise requirements language
* **Notes** - Elaboration and related information for the requirement

## 1.2 Requirement Language Keywords

In the requirements specified below, certain keywords have a specific meaning as they appear in the text. These keywords are defined as follows and must be capitalized whenever used in a manner intended to specify a behavior or requirement.

1. **MUST**: This word, or the terms "REQUIRED" or "SHALL", mean that the definition is an absolute requirement of the specification.

2. **MUST NOT**: This phrase, or the phrase "SHALL NOT", mean that the definition is an absolute prohibition of the specification.

3. **SHOULD**: This word, or the adjective "RECOMMENDED", mean that there may exist valid reasons in particular circumstances to ignore a particular item, but the full implications must be understood and carefully weighed before choosing a different course.

4. **SHOULD NOT**: This phrase, or the phrase "NOT RECOMMENDED" mean that there may exist valid reasons in particular circumstances when the particular behavior is acceptable or even useful, but the full implications should be understood and the case carefully weighed before implementing any behavior described with this label.

5. **MAY**: This word, or the adjective "OPTIONAL", mean that an item is truly optional.  One vendor may choose to include the item because a particular marketplace requires it or because the vendor feels that it enhances the product while another vendor may omit the same item. An implementation which does not include a particular option MUST be prepared to interoperate with another implementation which does include the option, though perhaps with reduced functionality. In the same vein an implementation which does include a particular option MUST be prepared to interoperate with another implementation which does not include the option (except, of course, for the feature the option provides.)

These definitions are derived from the IETF Best Current Practices Document #14.

## 1.3 Use Cases

The Navigation System is part of a larger system that involves a person or automated system ("the user") directing the operation of one or more robots. To provide context for the Navigation System, this section lists the expected interactions between the user and the robot system.  

## 1.3.1 Mapping Use Cases

The user will typically create a map of the area in which the robot is to navigate. This map identifies significant features of the environment, such as safety zones, known obstacles, walls, virtual lanes, etc. While the creation of the map itself is outside the scope of the Navigation System, the system is dependent on the map format(s). The map will need to be rich enough to support the Navigation System requirements listed in this document.

The following use case diagram shows an example of the kinds of operations provided by a mapping interface.

![Mapping Use Cases](./images/Mapping-Use-Cases.png)

## 1.3.2 Mission Planning Use Cases

Another area in which the user interacts with the system is in the creation of a mission plan for the robot. The user composes a sequence of primitive functions, such as **Navigate to Pose**, **Navigate to Area**, **Maintain Pose**, etc., into an overall plan. While mission planning is also outside the scope of the Navigation Stack, the mission plan format should be sufficient to meet the Navigation System requirements listed in this document.

![Mission Planning Use Cases](./images/Mission-Planning-Use-Cases.png)

## 1.3.2 Mission Execution Use Cases

The user will be able to initiate the execution of specific mission plans ("missions") and should also be able to view the status of the mission in progress, as well as cancel any mission that is in progress. In addition, the user may be required to provide the robot with its initial pose if the robot is not able to determine it automatically.

![Mission Execution Use Cases](./images/Mission-Execution-Use-Cases.png)

## 1.4 Architectural Components

The Navigation System is part of a larger robot software system. This document does not specify the architecture for the complete system, but simply names the various components and subsystems for the purpose of listing their associated requirements. 

The Navigation System has a *command chain*, where each level refines its command input into successively more specific operations for the lext level down, and *support modules* which are used by modules in the command chain. 

## 1.4.1 Command Chain

The command chain is the sequence of modules that comprise the chain of command from the user, at the top, to the robot, at the bottom.

* **Mission Planning** - Mission Planning provides an interface to the user to allow the user to create mission plans and assign them to robots for execution. A *Mission Plan* is a sequence of *Navigation Commands* along with associated information about how the commands should be carried out.
* **Mission Execution** - Mission Execution receives the Mission Plan and is responsible to execute the plan and report progress on its execution
* **Navigation System** - The Navigation System receives a segment of an overall plan to execute (a *Navigation Command*) and generates the control commands to the robot to carry it out.
* **Robot Interface** - The Robot Interface is an abstraction of the robot platform, providing the means for the Navigation System to control the robot, learn about its capabilities, and receive feedback from the robot. 

![Command Chain](./images/Context.png)

The Navigation System itself can be decomposed into two general responsibilities, *Planning*, and *Execution*.

* **Planning** - The Planning Module is responsible to execute Navigation Commands. To do so, this module can evaluate input maps and continually assess the robot's environment to plan motion and provide trajectories for the robot to follow to eventually achieve completion of the Navigation Command.
* **Control** - The Control Module is responsible to execute the trajectories provided by Planning, generating the control signals required to execute the Trajectory.

![Navigation System](./images/Navigation-System.png)

Decomposing the Navigation System, the overall command chain is as follows:

![Command Pipeline](./images/Command-Pipeline.png)

## 1.4.2 Support Modules

In addition to the main command chain, there are several supporting modules and subsystems required for a complete system. The implementation of these modules is outside the scope of the Navigation System. However, the inteface to these components is in scope and the associated requirements should be defined. 

* **Mapping** - The Mapping Subsystem generates maps that can be used by the Navigation System to plan the robot's motion. Maps are typically created in advance and are available to the Navigation System. A map can be updated to reflect changes in the environment. The frequency of these updates will vary among implementations.
* **Perception** - The Perception Subsystem utilizes sensors to develop an understanding of the dynamic environment around the robot. This information is available to the Navigation System in directing the robot's motion.  
* **Prediction** - The Prediction Subsystem anticipates future motion trajectories of any perceived objects.
* **Localization** - The Localization Subsystem provides the current location of the robot. 

In a complete robot system these modules are available to the core navigation modules (the command chain), as shown in the following diagram:

![Command Pipeline with Support](./images/With-Support-Modules.png)

To facilitate error recovery, each module in the chain, if it is unable to carry out its task, its able to propagate error information to its predecessor in the command chain.

## 1.5 Design Goals

The Navigation System designers should strive to meet the following high-level design goals:

* **Extensibility** - The Navigation Stack should be a *pluggable framework* to allow for other developers to easily extend the capabilities of the Navigation System.
* **Modularity** - The Navigation Stack should allow other developers to *easily replace components* with alternative implementations.
* **Generality** - The Navigation Stack should not introduce inherent limitations in the architectural blocks. For example, it should support multiple kinds of robots, not making assumptions about robot capabilities and limitations and should support various map types and orientations.
* **Performance** - *TODO: What are the performance goals?* 
* **Scalability** - *TODO: How low should the implementation scale?* *Specify a minimum platform?*
* *Other important design goals?*

In particular, there are several specific goals for the ROS 2 Navigation System with respect to improving the existing ROS navigation stack:

* The user should be able to plan complex missions which include a series of navigation commands, and the Navigation System should be able to accept those commands and execute them. 
* It should be possible to define navigation components for specific navigation commands. In executing a mission plan, the Navigation System would dynamically use the associated components for each navigation command. For example, a user may want to have components for classic point-A-to-point-B travel, but upon reaching point B, have a special components that control a series of maneuvers such as docking to a charging station or a conveyor belt. 
* The user should be able to specify different types of Robot drive types, such as Ackerman (automobile) steering, and Robot shapes (the current navigation stack is very limited in these areas).
* The Navigation System should be able to handle more map types and orientations, including outdoor and 2D+ terrain.

# 2.0 Requirements

This section lists the requirements for the Navigation System.

## 2.1 Implementation Constraints

There are various constraints on the development of the ROS 2 Navigation stack.

Id | Handle | Priority | Description | Notes
-- | ------ | -------- | ----------- | -----
IC001 | Developer's Guide | 1 | The Navigation System SHOULD be developed in accordance with the ROS 2 Devloper's Guide | [ROS 2 Developer's Guide](https://github.com/ros2/ros2/wiki/Developer-Guide)
IC002 | Implementation Language.C++.Version | 1 | Developers SHALL assume the availability of C++14 language features | Per the ROS 2 Developer's Guide
IC003 | Implementation Language.C++.API Preference | 1 | Developers SHOULD prefer standard C++, then Boost, then custom code, in that order. | Boost may be used if equivalent functionality is not already available in the C++ standard library
IC004 | Implementation Language.C++.Supported Compilers.g++ | 1 | The Navigation System code SHALL compile with gcc 5.4 or newer
IC004 | Implementation Language.C++.Supported Compilers.Clang | 1 | The Navigation System code SHALL compile with Clang, version *x*
IC004 | Implementation Language.C++.Supported Compilers.Intel C++ Compiler | 1 | The Navigation System code SHOULD compile with the Intel C++ Compiler, version *x* | Could be useful for optimization purposes
IC005 | Implementation Language.Python.Version | 1 | Any Python code developed for the Navigation System MUST use Python 3
IC006 | Implementation Language.GUI | 1 | Any GUIs developed as part of the Navigation System MUST use the Qt library, via C++ or Python (PyQt) | *Which version?*
IC006 | Implementation Language.GUI.QML | 1 | Any GUIs developed as part of the Navigation System MAY use QML
IC007 | ROS2.Version | 1 | The Navigation System WILL be developed against the latest stable version of the ROS 2 stack | *What is the current latest version?*

## 2.2 Target Platforms

The Navigation System will run on the latest versions of the operating systems supported by the core ROS 2 code.

Id | Handle | Priority | Description | Notes
-- | ------ | -------- | ----------- | -----
PLT001 | Target Platforms.Operating Systems.Ubuntu | 1 | The Navigation System MUST support Ubuntu Desktop 16.04 and Ubuntu Desktop 18.04
PLT002 | Target Platforms.Operating Systems.MacOS | 1 | The Navigation System MUST support MacOS 10.13 (High Sierra) and MacOS 10.14 (Mohave)
PLT003 | Target Platforms.Operating Systems.Windows | 1 | The Navigation System MUST support Windows 10 Professional
PLT004 | Target Platforms.Operating Systems.Clear Linux | 1 | The Navigation System SHOULD support the latest version of Intel's Clear Linux distribution | What is the latest version number?
PLT005 | Target Platforms.CPU.Word Size | 1 | The Navigation System SHALL support 64-bit processors | Don't assume a specific pointer size
PLT006 | Target Platforms.Minimum Platform | 1 | *Should we specify a minimum target platform?* *Or, should this be expressed as minimum platform requirements?*

## 2.3 Command Chain Modules

This section lists the requirements for the core command chain modules in the Navigation System.

### 2.3.1 Mission Planning

TODO: 

The Mission Planning subsystem is outside the scope of the Navigation System. However, the data format, or messages, required to interact with the subsystem area within scope and must be specified as the Navigation System needs to receive this information to be able to execute the mission plans.

A complete system, which incorporates the Navigation System, would also have a Mission Planning subsystem that generates a Mission Plan from input provided by the User. The Mission Plan is a sequence of primitive operations and any associated policy about how the plan should be carried out. The design and implementation of the subsystem that creates the Mission Plan is outside the scope of the Navigation System, but its output, the Mission Plan, may have an impact on capabilities that the Navigation System must provide. 

*TODO: Rephrase using "mission plan" terminology rather than taking the robot's perspective*

Id | Handle | Priority | Description | Notes
-- | ------ | -------- | ----------- | -----
MP001 | Mission Planning.Primitives | 1 | The Mission Plan MUST be able to express the plan as a sequence of the following Primitives (MP002 - MP010)
MP002 | Mission Planning.Primitives.Navigate to Pose | 1 | The Navigation System MUST be able to navigate the Robot from its current location to a specified destination pose.
MP003 | Mission Planning.Primitives.Navigate to Area | 1 | The Navigation System MUST be able to navigate the Robot from its current location to a specified area. | *TODO: How to define "area"?*
MP004 | Mission Planning.Primitives.Enqueue | 1 | The Navigation System MUST be able to navigate the Robot from its current location to a position behind another specified robot. 
MP005 | Mission Planning.Primitives.Follow | 1 | The Navigation System MUST be able to direct the Robot to follow another specified robot. | This one doesn't have a completion state (reaching the goal), unless it specifies some more information such as "follow until destination reached"
MP006 | Mission Planning.Primitives.Orbit | 1 | *TODO: How is this defined?* | Similar to Follow in that it doesn't complete (unless it specifies time)
MP007 | Mission Planning.Primitives.Maintain Pose | 1 | The Navigation System MUST be able to maintain its current pose | Could be indefinite or time-based
MP008 | Mission Planning.Primitives.Park | 1 | *TODO: What does parking mean? Some low-power state?*
MP009 | Mission Planning.Primitives.Enter Elevator | 1 | The Navigation System MUST be able to navigate the Robot from its current location onto an elevator. | *TODO: Would the elevator door be automated?*
MP010 | Mission Planning.Primitives.Exit Elevator | 1 | The Navigation System MUST be able to navigate the Robot from its position in an elevator to a specified location outside the elevator. 
MP011 | Mission Planning.Policy | 1 | The Mission Plan MUST be able to express the following policy associated with how to carry out the mission (MP012 - MP015)| *What are the important qualities for how the primitives should be carried out? Should there be per-primitive policy?*
MP012 | Mission Planning.Policy.Timeliness | 1 | The Navigation System MUST respect any timeliness constraints in the Mission Plan | *TODO: Is it important to specify by when the mission should be completed, for example?*
MP013 | Mission Planning.Policy.Safety Constraints | 1 | The Navigation System MUST respect any safety constraints in the Mission Plan | *TODO: Is there a set of global safety-related policy for the robot to follow?*
MP014 | Mission Planning.Policy.Safety Constraints.Maximum Speed | 1 | The Navigation System MUST respect the maximum speed specified in the Mission Plan 
MP015 | Mission Planning.Policy.Safety Constraints.Minimum Safety Buffer | 1 | The Navigation System MUST maintain a minimum safe distance between the robot and other objects at all times, as specified int he Mission Plan | Configurable safety zone around the robot. *TODO: Variable with speed? Differ in each direction?*

*TODO: How to handle reverse direction? Does each primitive include this information?*

### 2.3.2 Mission Execution

The Mission Execution module has the responsibility to execute a provided mission. It provides each successive primitive operation to the global planner, monitoring and reporting progress towards the goal.

Id | Handle | Priority | Description | Notes
-- | ------ | -------- | ----------- | -----
ME001 | Mission Execution.Inputs.Mission Plan | 1 | The Mission Execution module MUST accept the Mission Plan to execute.
ME002 | Mission Execution.Inputs.Commands.Execute Mission | 1 | The Mission Execution module MUST execute the provided Mission Plan, respecting any specified constraints.
ME003 | Mission Execution.Inputs.Commands.Cancel Mission | 1 | The Mission Execution module MUST be able to interrupt the Robot's navigation and cancel the current mission.
ME004 | Mission Execution.Select Planners | 1 | The Mission Execution module SHOULD select the best planners to use for each mission primitive | Can dynamically utilize planners
ME005 | Mission Execution.Outputs | 1 | At each stage of the Mission Plan, the Mission Execution module SHALL output the next primitive to execute and WILL monitor for completion of each primitive before sending the next. | Tracks completion of the mission and sequences the operations. *TODO: What about primitives that don't complete? Would they be defined with a time element so that they can complete?*
ME006 | Mission Execution.Feedback.Outputs.Notify Progress | 1 | The Mission Execution module MUST provide progress notifications on the execution of the mission, including mission completion. | The current stack, once it reaches a goal, publishes a message. This would be a generalization of that feature. We can determine the elapsed time using the message timestamp.
ME007 | Mission Execution.Logging | 1 | The Mission Execution module SHOULD log activity for later analysis. | In case of forensic analysis of a safety event, for example.
ME008 | Mission Execution.Feedback.Inputs.Error Notification.Automatic Error Recovery | 1 | Upon receipt of a downstream failure (from a Planner), the Mission Execution module SHOULD attempt to automatically recover from the error | For example, if the robot gets stuck, the Mission Execution module could have the robot perform a recovery maneuver of some kind, before continuing with execution of the mission plan.
ME009 | Mission Execution.Feedback.Outputs.Error Information | 1 | If the Mission Execution module is unable to execute the mission, it MUST report the failure | This would be received by the higher-level software. Perhaps a remote operating center where the remote operator "rescues" the robot. 
ME010 | Mission Execution.Safe State Upon Failure | 1 | If the Mission Execution module is unable to execute the mission, it MUST direct the robot to a safe state. | *TODO: Define "safe state"* The failure could be for a variety of reasons - sensor failures, algorithmic failure, a collision, etc.

### 2.3.3 Routing

The Routing Module computes the intended route for the robot, typically using a map, the robot's initial pose, and the desired primitive to execute. 

Id | Handle | Priority | Description | Notes
-- | ------ | -------- | ----------- | -----
GP001 | Routing | 1 | The Navigation System SHOULD have a Routing Module which generates the route for the robot to follow for the specified primitive, respecting any policy guidance contained in the Mission Plan
GP002 | Routing.Inputs.Map | 1 | The Routing Module SHALL have a map available that describes the environment | Assumes the map has already been created and available as an input
GP003 | Routing.Inputs.Current Pose | 1 | The Routing Module MUST have the robot's current pose | The pose could be be provided manually or automatically determined (outside of this module)
GP004 | Routing.Inputs.Primitive to Execute | 1 | The Routing Module SHALL have the primitive operation to execute
GP005 | Routing.Inputs.Policy | 1 | The Routing Module SHALL have policy guidance associated with the primitive operation to execute | This could be global policy and/or per-primitive policy
GP009 | Routing.Inputs.Policy.Conventions | 1 | *TODO: Should there be a specification of conventions for the robot to follow? Navigate on the right side of a path, for example.*
GP012 | Routing.Outputs.Route | 1 | The Path Planning Module should output a route that the robot is to follow to execute the primitive
GP013 | Routing.Outputs.Policy Attributes | 1 | *TODO: Should there be attributes associated, such as time to be at particular waypoints?*

*TODO: Routing Plugins*
GP006 | Routing.Multiple Planners | 1 | As part of the Navigation System, there SHOULD be multiple global planners available and chosen depending on the primite to be executed | Dynamic selection of global planners
GP007 | Routing.High-End Planner | 1 | The Navigation System SHOULD implement a state-of-the-art planner | Can serve as an example to others developing global planners
GP008 | Routing.Low-End Planner | 1 | The Navigation System SHOULD implement a planner for low-compute targets | Can serve as an example to others developing global planners
*TODO: Well documented, examples*

GP010 | Routing.Feedback.Inputs | 1 | The Planner MUST receive error input from the Planning Module so that it can attempt to recover from planning failures | For example, if a planner can't execute the route because it is blocked, the global planner has an opportunity to define an alternative route
GP011 | Routing.Feedback.Outputs | 1 | If the Planner is unable to define a route to its goal state, it SHALL report the error on its feedback output

### 2.3.4 Planning

The Planning Module receives the route from the Routing Module and is responsible to carry it out. To do so, it evaluates the dynamic environment using input from the Perception Subsystem, possibly making local adjustments to the route, such as when avoiding collisions with objects crossing its path. 

Id | Handle | Priority | Description | Notes
-- | ------ | -------- | ----------- | -----
LP001 | Planning | 1 | The Navigation System SHOULD have a Planning Module which carries out the route provided by the Planner
LP002 | Planning.Inputs.Route | 1 | The Planning Module MUST receive the Route from the Path Planning Module.
LP003 | Planning.Inputs.Route.Negative Velocity | 1 | To support reverse motion, the Planning Module MUST support negative velocity values
LP004 | Planning.Inputs.Policy | 1 | The Planning Module MUST receive any policy information which could inform its planning
LP005 | Planning.Inputs.Sensory Input | 1 | The Planning Module MUST receive object-level data from the Sensory Subsystem 
LP016 | Planning.Outputs | 1 | The Planning Module interfaces to the robot to command its actuators. | TBD if this is through an abstraction of the robot or directly to a vendor's interface.

LP006 | Planning.Collision Avoidance.Avoid Stationary Objects | 1 | The Robot MUST avoid colliding into stationary objects in its environment | Fixed parts of the environment, such as walls or shelving. 
LP007 | Planning.Collision Avoidance.Avoid Stationary Objects.*Qualities* | 1 | *TODO: How close can it come? Does it depend on the robot's speed?*
LP008 | Planning.Collision Avoidance.Avoid Moving Objects | 1 | The Robot MUST avoid colliding into moving objects that intercept its path | Such as people and other robots
LP009 | Planning.Collision Avoidance.Avoid Moving Objects.*Qualities* | 1 | *TODO: How close can it come? Does it depend on the robot speed?*
LP010 | Planning.Collision Avoidance.Parameters | 1 | *TODO: Any specific parameters foer the collision avoidance? Perhaps different global values for object-specific properties in the environment.*
LP011 | Planning.Collision Detection | 1 | *TODO: What module first detects if there has been a collision?* Some way of detecting if a collision has occurred within some low latency (50ms?). This could be additional HW such as bumpers (like Turtlebot has), or other sensor input such as wheel feedback
LP012 | Planning.Collision Reporting | 1 | *TODO: What happens in the error/exception reporting chain to report a collision?*
LP013 | Planning.Collision Recovery | 1 | The Planning Module MUST provide a way to recover from collisions | Like back up slowly, notify user, etc., which could be configurable based on the robot/environment/application
LP014 | Planning.Feedback.Inputs | 1 | TODO | Could be failures from the sensors, for example
LP015 | Planning.Feedback.Outputs | 1 | TODO | Could provide failure to execute the route. Perhaps the passageway is blocked, for example

### 2.3.5 Control

*TODO: The Control Module ... Bottom half of the Local Planner (Planning)*

Id | Handle | Priority | Description | Notes
-- | ------ | -------- | ----------- | -----
CTL001 | Control | 1 | The Navigation System SHOULD have a Control Module which TODO

### 2.3.6 Robot Interface

*TODO: What to say about the robot interface?* *Should there be a robot abstraction?* *Perhaps this would be a convenient place to put some safety-related functionality*
There should be a uniform interface to the various supported robots. This may require a layer on top of the vendor-specific interface(s).

Id | Handle | Priority | Description | Notes
-- | ------ | -------- | ----------- | -----
ROB001 | Robot Interface.Attributes | 1 | Holonomicity, max/min speeds and accelerations, etc. 
ROB002 | Robot Interface.Dynamic Switching | 1 | Can the robot dynamically change attributes?
ROB003 | Robot Interface.Safety.Limited Parameters | 1 | A list of parameters used to limit certain circumstances and provide the hooks for users to set those values if they want
ROB004 | Robot Interface.Safety.Speed Limiting | 1 | TODO
ROB005 | Robot Interface.Safety.Force Limiting | 1 | TODO
ROB006 | Robot Interface.EMO Button | 1 | TODO
ROB007 | Robot Interface.Feedback.Outputs | 1 | TODO

## 2.4 Support Modules

There are a few support modules and subsystems that are not part of the Navigation System proper, but are necessary components in a complete system. The Navigation System depends on the data interfaces to these components. This section describes the requirements and assumptions of these interface.

### 2.4.1 Mapping

The map data format should be capable of describing common indoor and outdoor environments.

Id | Handle | Priority | Description | Notes
-- | ------ | -------- | ----------- | -----
MAP001 | Mapping | 1 | The Mapping System SHALL provide map information to the Navigation System | So that the Mapping System has a basic understanding of its environment. Used in global planning
MAP002 | Mapping.Dimensionality.2D | 1 | The Mapping System MUST provide 2D map information
MAP003 | Mapping.Dimensionality.2D+ | 1 | The Mapping System MUST provide 2D+ map information | *TODO: How is this defined?*
MAP004 | Mapping.Dimensionality.3D | 1 | The Mapping System SHOULD provide 3D map information | *TODO: Use voxel-based?*
MAP005 | Mapping.Multiple Maps Per Environment | 1 | The Mapping system SHOULD provide multiple maps of the same environment. | *TODO: Different scales and elevations?*
MAP006 | Mapping.Dynamic Updates | 1 | The Mapping System SHOULD provide real-time updates of map information | For example, to create a keep-out zone in the event of a spill hazard.
MAP007 | Mapping.Data Model.Traversable Surface | 1 | Planar (2D) surface (street, floor). 
MAP008 | Mapping.Data Model.Traversable Surface.Planarity | 1 | The map data format SHALL be capable of describing the planarity of traversable surfaces | Uneven ground
MAP009 | Mapping.Data Model.Safety Zone | 1 | Specify limitations to be respected while the robot is in this zone. Speed, (increased) distance to people.
MAP010 | Mapping.Data Model.Safety Zone.Keep Out Zone | 1 | Robot MUST NOT navigate through a keep-out zone
MAP011 | Mapping.Data Model.Safety Zone.Intersection | 1 | May want to slow down (or even stop), for example, when approaching an intersection | Robots can slow down when coming to an intersection in order to ensure that they can see anyone coming and other people/robots can see them
MAP012 | Mapping.Data Model.Lanes | 1 | Able to specify virtual lanes. Prefer the specified lanes (such as in a warehouse)
MAP013 | Mapping.Data Model.Building Levels | 1 | Able to specify single and multi-level buildings
MAP014 | Mapping.Data Model.Building Levels.Level Connecting Features | 1 | Able to specify level connecting features, such as elevators, stairways, ramps.
MAP015 | Mapping.Data Model.Unknown Space | 1 | To be dynamically mapped? In global maps, it can be areas beyond the edge of the map, or areas within the center of the map for which we didn't have any observations during map building. In the current system, this is the "dark grey" portion of the map (free space is light grey, obstacles are black)
MAP016 | Mapping.Data Model.Extensibility | 1 | *TODO: Map attributes. Layers concept?*

### 2.4.2 Perception

The Navigation System requires dyanamic information about objects in its environment. This information is provided by a Sensory Subsystem that is outside the scope of the Navigation System. This information would typically be provided by a set of sensors on the robot and a sensor fusion capability. This section lists the requirements on the data that the Sensory Subsystem provides. 

Id | Handle | Priority | Description | Notes
-- | ------ | -------- | ----------- | -----
SEN001 | Sensing | 1 | The Sensory Subsystem SHALL provide information about the dynamic environment of the robot | Info sufficient to carry out the Navigation System requirements
SEN002 | Sensing.Latency | 1 | TODO
SEN003 | Sensing.Environment Model | 1 | *TODO: What, specifically, should be in the environment model?*
SEN004 | Sensing.Environment Model.Object Prediction | 1 | The Sensing Subsystem SHOULD provide an object prediction capability that projects the position of the objects into the future. | One of the biggest shortcomings of the current system is the inability to model/predict where obstacles will be in the future. This leads to collisions with other moving objects
SEN005 | Sensing.Environment Model.Object Prediction.Time Horizon | 1 | *TODO: How far into the future should the object prediction work?*

### 2.4.3 Prediction

The Prediction Subsystem TODO

Id | Handle | Priority | Description | Notes
-- | ------ | -------- | ----------- | -----
PRE001 | Localization.Robot Pose | 1 | TDOO

### 2.4.4 Localization

The Navigation System requires the Robot's current pose, provided by an external Localization module. This section lists the requirements for the information from the Localization module.

Id | Handle | Priority | Description | Notes
-- | ------ | -------- | ----------- | -----
LOC001 | Localization.Robot Pose | 1 | The Localization module MUST provide the robot's current pose to the Navigation System | This could be manual or as a result of automatic localization; the Navigation System wouldn't know either way 
LOC002 | Localization.Robot Pose.Accuracy | 1 | *TODO: What are the requirements on the accuracy of this pose?*

 +1 1  
 
TODO:

DONE: (mikefurgeson) Should we try to move away from calling this "static map" -- a common use case would be running navigation at the same time as mapping -- to explore the environment. In this case, the map isn't "static" but rather slowly changing. In particular, the important part is that the map must have some "constant reference frame" even as it changes over time (for instance, you can arbitrarily move 0,0 and expect things to still work, but if you add onto the map at runtime, the system should be able to handle it).

(mikeferguson) Ideally, this module should also be extremely well commented and documented, such that it can be used as a teaching tool -- I presume it will be a fairly straight forward A* implementation, which means it can be easily understood by students and adapted during various coursework.

(mike furgeson) In supporting this, I think you'll find that you also want to make it easy to "reset or disable" a planner. Most of these planners may have large memory usage, and for efficiency may not want to recreate them for each plan -- if you have multiple running at once, each taking up memory, you might actually run out of memory quickly. Having an easy API to say "hey, not using this planner for a while, have it release resources" would be very useful.

Mapping: (mikefurgeson) Any thoughts around tiling? Especially for out-of-memory situations? Yes, we do this for Automotive

Localization:  (mikefurgeson) This won't be exact -- so it should also provide an estimate of the accuracy (current navigation stack can't take that into account, but future local planners certainly could). The current message is PoseWithCovariance to accomplish this in ROS1.

Turtlebot image: (Carl)

MP002: (Carl) What if the planner can't find a path? Then we can't meet this requirement. Should be rephrased to allow for failure (and the resulting reporting of failure)

MP004,MP005: (Carl) This requires a much more complicated understanding of the world than I think we want to incorporate into the navstack. It implies we can identify what things in the environment are robots and distinguish a particular one even as it moves around in traffic, possibly goes out of view etc. (mikefurgeson) I concur -- this should provide a tool for generic navigation, and higher level "applications" would then call into the navigation stack. (Mohammad) Are we planning to implement object detection and tracking to accomplish this task? Or does the Nav stack gets a series of waypoints from other supporting modules?

MP006: (Carl) Orbit what? I can see potentially orbitting a fixed position in the world. Orbitting an object seems outside our scope.
 
MP007: (Carl) Could this just be reduced to not having a specified objective? What would the robot do if it isn't given a wait primitive? (Mohammad) "Wait" should be changed to "HoldPose". For example: the robot is on an inclined plane and the user wants the robot to stay at its current pose. @crdelsey If the HoldPose primitive is not given the robot would role in this example. HoldPose may also be considered to be out of scope for Nav stack. The robot's base controller should take care of holding the pose.

MP008: (Carl) Going to a low power state seems out of scope for the navstack.

MP001: (mikefurgeson) I understand the desire to be able to "plan several legs of a trip at once, so if one is impassable you don't start the trip at all", but I feel like that is fairly incongruent with how actions typically are implemented. I know actionlib isn't implemented in ROS2 yet, but the issue I see arising is: how do you cancel part way through?

MP009: (Carl) Why is this a special primitive? Can't "go to area" suffice? (Me) I'm not sure. We'd have to capture that the floor isn't always present, at least. It can't be freely navigated into.

MP010: (Carl) Can't we just use go to location/area to get out of the elevator. (Me) I think we need to consider how to handle elevators and other kinds of lifts, where the mechanism has various states. If there's an external controller that's handling the elevator doors, etc., the Execution mechanism for the robot needs to at least feedback that it's ready to enter (or exit) an elevator and wait for the go-head. I think we should mock up some example mission plans and work through these issues.

MP012: (Carl) How would this work? What if we can't meet the time requirement? Do we need to calculate out the ETA before starting so we can throw an error as soon as the command is given? What happens if the time constraint is much greater than required? Do we slow down or wait somewhere? (Me) Good questions. For me, the first question is whether it is important to introduce time-related requirements into the mission plan or not. If so, we'll have to address the issues that you mention. Would be good to get input from users.

Mapping Use Cases: (Mohammad) Suggestion: "The user will typically either manually create a map or uses SLAM algorithm to create a map of the area in which the robot is to navigate."
Suggestion: Changing the phrase "known obstacles" to "known static obstacles".

DONE: Mission Planning Use Cases: (Mohammad)  Changing the phrase "Navigate to Position" to either "Navigate to Position and Orientation" or "Navigate to Pose"

DONE: Mission Execution Use Cases: (Mohammad) We might want to change the phrase "initial location" to "initial pose" which includes both position and orientation.

DONE: GP003: (Mohammad) The Global Planner "MUST" have the robot's current pose.

MP005: (Mohammad)  Are we planning to implement object detection and tracking to accomplish this task? Or does the Nav stack gets a series of waypoints from other supporting modules?

DONE: MP007: (Mohammad) "Wait" should be changed to "HoldPose". For example: the robot is on an inclined plane and the user wants the robot to stay at its current pose. @crdelsey If the HoldPose primitive is not given the robot would role in this example.