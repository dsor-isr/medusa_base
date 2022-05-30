---
title: PathFollowingNode
summary: Path Following Node, where the magic happens. 

---

# PathFollowingNode



Path Following Node, where the magic happens.  [More...](#detailed-description)


`#include <PathFollowingNode.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[PathFollowingNode](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowingNode/#function-pathfollowingnode)**(ros::NodeHandle * nh, ros::NodeHandle * nh_p)<br>The constructor of the path following node.  |
| | **[~PathFollowingNode](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowingNode/#function-~pathfollowingnode)**()<br>The destructor of the path following node (where the subscribers, publishers and services are terminated)  |

## Detailed Description

```cpp
class PathFollowingNode;
```

Path Following Node, where the magic happens. 

**Author**: 

  * [Marcelo](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/) Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 
## Public Functions Documentation

### function PathFollowingNode

```cpp
PathFollowingNode(
    ros::NodeHandle * nh,
    ros::NodeHandle * nh_p
)
```

The constructor of the path following node. 

**Parameters**: 

  * **nh** The public ROS node handle 
  * **nh_p** The private ROS node handle
  * **nh** Public ros node handle 
  * **nh_p** Private ros node handle 


The Path Following Node constructor.


### function ~PathFollowingNode

```cpp
~PathFollowingNode()
```

The destructor of the path following node (where the subscribers, publishers and services are terminated) 

Node class destructor. 


-------------------------------

Updated on 2022-05-30 at 08:04:30 +0000