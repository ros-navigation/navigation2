# nav2_msgs

The `nav2_msgs` package is a set of messages, services, and actions for the `navigation2` stack. The `navigation2` stack still makes use of `nav_msgs` from ROS (1) Navigation.

The usage of messages and services in ROS 2 is slightly different. See a comparison below:

```
// ROS 2 Style
#include "nav_msgs/msg/path.hpp"

```

```
// ROS 1 Style
#include "nav_msgs/Path.h"
```

As you can see, in ROS 2, messages have an `.hpp` ending and are located in the directory of the type (msg, srv, action). In addition, all header files are lower-snake case. Where `TaskStatus` in ROS 1/2 would be:

```
// ROS 2 Style
#include "nav2_msgs/msg/task_status.hpp"

```

```
// ROS 1 Style
#include "nav2_msgs/TaskStatus.h"
```

In ROS 2, regardless of the file name, the outputted messages are lower case, snake cased.
