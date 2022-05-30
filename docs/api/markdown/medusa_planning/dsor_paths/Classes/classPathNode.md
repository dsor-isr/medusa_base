---
title: PathNode
summary: Implementation of the PathNode. Creates a Path, adds elements to the path and publishes the path data when listening to the path parameter gamma. 

---

# PathNode



Implementation of the [PathNode](). Creates a [Path](), adds elements to the path and publishes the path data when listening to the path parameter gamma.  [More...](#detailed-description)


`#include <PathNode.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[PathNode](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathNode/#function-pathnode)**(ros::NodeHandle * nh, ros::NodeHandle * nh_p)<br>Constructor of the [PathNode](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathNode/) class.  |
| | **[~PathNode](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathNode/#function-~pathnode)**()<br>Destructor of the [PathNode](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathNode/) class.  |

## Detailed Description

```cpp
class PathNode;
```

Implementation of the [PathNode](). Creates a [Path](), adds elements to the path and publishes the path data when listening to the path parameter gamma. 

**Author**: 

  * Marcelo Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 
## Public Functions Documentation

### function PathNode

```cpp
PathNode(
    ros::NodeHandle * nh,
    ros::NodeHandle * nh_p
)
```

Constructor of the [PathNode](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathNode/) class. 

**Parameters**: 

  * **nh** Pointer to a public ROS NodeHandle 
  * **nh_p** Pointer to a private ROS NodeHandle
  * **nh** A pointer to the public ROS node handle 
  * **nh_p** A pointer to the private ROS node handle 


[PathNode](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathNode/) constructor. Receives the ROS node handles as inputs and initializes the subscribers, publishers, timers, parameters, etc...


### function ~PathNode

```cpp
~PathNode()
```

Destructor of the [PathNode](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathNode/) class. 

Class destructor. Called when deleting the class object. 


-------------------------------

Updated on 2022-05-30 at 08:04:27 +0000