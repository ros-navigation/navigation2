Architecture of `launch`
========================

`launch` is designed to provide core features like describing actions (e.g. executing a process or including another launch description), generating events, introspecting launch descriptions, and executing launch descriptions.
At the same time, it provides extension points so that the set of things that these core features can operate on, or integrate with, can be expanded with additional packages.

Launch Entities and Launch Descriptions
---------------------------------------

The main object in `launch` is the :class:`launch.LaunchDescriptionEntity`, from which other entities that are "launched" inherit.
This class, or more specifically classes derived from this class, are responsible for capturing the system architect's (a.k.a. the user's) intent for how the system should be launched, as well as how `launch` itself should react to asynchronous events in the system during launch.
A launch description entity has its :meth:`launch.LaunchDescriptionEntity.visit` method called during "launching", and has any of the "describe" methods called during "introspection".
It may also provide a :class:`asyncio.Future` with the :meth:`launch.LaunchDescriptionEntity.get_asyncio_future` method, if it has on-going asynchronous activity after returning from visit.

When visited, entities may yield additional entities to be visited, and this pattern is used from the "root" of the launch, where a special entity called :class:`launch.LaunchDescription` is provided to start the launch process.

The :class:`launch.LaunchDescription` class encapsulates the intent of the user as a list of discrete :class:`launch.Action`'s, which are also derived from :class:`launch.LaunchDescriptionEntity`.
As "launch description entities" themselves, these "actions" can either be introspected for analysis without performing the side effects, or the actions can be executed, usually in response to an event in the launch system.

Additionally, launch descriptions, and the actions that they contain, can have references to :class:`launch.Substitution`'s within them.
These substitutions are things that can be evaluated during launch and can be used to do various things like: get a launch configuration, get an environment variable, or evaluate arbitrary Python expressions.

Launch descriptions, and the actions contained therein, can either be introspected directly or launched by a :class:`launch.LaunchService`.
A launch service is a long running activity that handles the event loop and dispatches actions.

Actions
-------

The aforementioned actions allow the user to express various intentions, and the set of available actions to the user can also be extended by other packages, allowing for domain specific actions.

Actions can have direct side effects (e.g. run a process or set a configuration variable) and as well they can yield additional actions.
The latter can be used to create "syntactic sugar" actions which simply yield more verbose actions.

Actions may also have arguments, which can affect the behavior of the actions.
These arguments are where :class:`launch.Substitution`'s can be used to provide more flexibility when describing reusable launch descriptions.

Basic Actions
^^^^^^^^^^^^^

`launch` provides the foundational actions on which other more sophisticated actions may be built.
This is a non-exhaustive list of actions that `launch` may provide:

- :class:`launch.actions.IncludeLaunchDescription`

  - This action will include another launch description as if it had been copy-pasted to the location of the include action.

- :class:`launch.actions.SetLaunchConfiguration`

  - This action will set a :class:`launch.LaunchConfiguration` to a specified value, creating it if it doesn't already exist.
  - These launch configurations can be accessed by any action via a substitution, but are scoped by default.

- :class:`launch.actions.DeclareLaunchDescriptionArgument`

  - This action will declare a launch description argument, which can have a name, default value, and documentation.
  - The argument will be exposed via a command line option for a root launch description, or as action configurations to the include launch description action for the included launch description.

- :class:`launch.actions.SetEnvironmentVariable`

  - This action will set an environment variable by name.

- :class:`launch.actions.GroupAction`

  - This action will yield other actions, but can be associated with conditionals (allowing you to use the conditional on the group action rather than on each sub-action individually) and can optionally scope the launch configurations.

- :class:`launch.actions.TimerAction`

  - This action will yield other actions after a period of time has passed without being canceled.

- :class:`launch.actions.ExecuteProcess`

  - This action will execute a process given its path and arguments, and optionally other things like working directory or environment variables.

- :class:`launch.actions.RegisterEventHandler`

  - This action will register an :class:`launch.EventHandler` class, which takes a user defined lambda to handle some event.
  - It could be any event, a subset of events, or one specific event.

- :class:`launch.actions.UnregisterEventHandler`

  - This action will remove a previously registered event.

- :class:`launch.actions.EmitEvent`

  - This action will emit an :class:`launch.Event` based class, causing all registered event handlers that match it to be called.

- :class:`launch.actions.LogInfo`:

  - This action will log a user defined message to the logger, other variants (e.g. `LogWarn`) could also exist.

- :class:`launch.actions.RaiseError`

  - This action will stop execution of the launch system and provide a user defined error message.

More actions can always be defined via extension, and there may even be additional actions defined by `launch` itself, but they are more situational and would likely be built on top of the above actions anyways.

Base Action
^^^^^^^^^^^

All actions need to inherit from the :class:`launch.Action` base class, so that some common interface is available to the launch system when interacting with actions defined by external packages.
Since the base action class is a first class element in a launch description it also inherits from :class:`launch.LaunchDescriptionEntity`, which is the polymorphic type used when iterating over the elements in a launch description.

Also, the base action has a few features common to all actions, such as some introspection utilities, and the ability to be associated with a single :class:`launch.Condition`, like the :class:`launch.IfCondition` class or the :class:`launch.UnlessCondition` class.

The action configurations are supplied when the user uses an action and can be used to pass "arguments" to the action in order to influence its behavior, e.g. this is how you would pass the path to the executable in the execute process action.

If an action is associated with a condition, that condition is evaluated to determine if the action is executed or not.
Even if the associated action evaluates to false the action will be available for introspection.

Substitutions
-------------

A substitution is something that cannot, or should not, be evaluated until it's time to execute the launch description that they are used in.
There are many possible variations of a substitution, but here are some of the core ones implemented by `launch` (all of which inherit from :class:`launch.Substitution`):

- :class:`launch.substitutions.Text`

  - This substitution simply returns the given string when evaluated.
  - It is usually used to wrap literals in the launch description so they can be concatenated with other substitutions.

- :class:`launch.substitutions.PythonExpression`

  - This substitution will evaluate a python expression and get the result as a string.

- :class:`launch.substitutions.LaunchConfiguration`

  - This substitution gets a launch configuration value, as a string, by name.

- :class:`launch.substitutions.LaunchDescriptionArgument`

  - This substitution gets the value of a launch description argument, as a string, by name.

- :class:`launch.substitutions.LocalSubstitution`

  - This substitution gets a "local" variable out of the context. This is a mechanism that allows a "parent" action to pass information to sub actions.
  - As an example, consider this pseudo code example `OnShutdown(actions=LogInfo(msg=["shutdown due to: ", LocalSubstitution(expression='event.reason')]))`, which assumes that `OnShutdown` will put the shutdown event in the locals before `LogInfo` is visited.

- :class:`launch.substitutions.EnvironmentVariable`

  - This substitution gets an environment variable value, as a string, by name.

- :class:`launch.substitutions.FindExecutable`

  - This substitution locates the full path to an executable on the PATH if it exists.

The base substitution class provides some common introspection interfaces (which the specific derived substitutions may influence).

The Launch Service
------------------

The launch service is responsible for processing emitted events, dispatching them to event handlers, and executing actions as needed.
The launch service offers three main services:

- include a launch description

  - can be called from any thread

- run event loop
- shutdown

  - cancels any running actions and event handlers
  - then breaks the event loop if running
  - can be called from any thread

A typical use case would be:

- create a launch description (programmatically or based on a markup file)
- create a launch service
- include the launch description in the launch service
- add a signal handler for SIGINT that calls shutdown on the launch service
- run the event loop on the launch service

Additionally you could host some SOA (like REST, SOAP, ROS Services, etc...) server in another thread, which would provide a variety of services, all of which would end up including a launch description in the launch service asynchronously or calling shutdown on the launch service asynchronously.
Remember that a launch description can contain actions to register event handlers, emit events, run processes, etc.
So being able to include arbitrary launch descriptions asynchronously is the only feature you require to do most things dynamically while the launch service is running.

Event Handlers
--------------

Event handlers are represented with the :class:`launch.EventHandler` base class.
Event handlers define two main methods, the :meth:`launch.EventHandler.matches` method and the :meth:`launch.EventHandler.handle` method.
The matches method gets the event as input and must return `True` if the event handler matches that event, or `False` otherwise.
The handle method gets the event and launch context as input, and can optionally (in addition to any side effects) return a list of :class:`launch.LaunchDescriptionEntity` objects to be visited by the launch service.

Event handlers do not inherit from :class:`launch.LaunchDescriptionEntity`, but can similarly be "visited" for each event processed by the launch service, seeing if `matches` returns `True` and if so following up with a call to `handle`, then visiting each of the actions returned by `handle`, depth-first.

Extension Points
----------------

In order to allow customization of how `launch` is used in specific domains, extension of the core categories of features is provided.
External Python packages, through extension points, may add:

- new actions

  - must directly or indirectly inherit from :class:`launch.Action`

- new events

  - must directly or indirectly inherit from :class:`launch.Event`

- new substitutions

  - must directly or indirectly inherit from :class:`launch.Substitution`

- kinds of entities in the launch description

  - must directly or indirectly inherit from :class:`launch.LaunchDescriptionEntity`

In the future, more traditional extensions (like with `setuptools`' `entry_point` feature) may be available via the launch service, e.g. the ability to include some extra entities and event handlers before the launch description is included.
