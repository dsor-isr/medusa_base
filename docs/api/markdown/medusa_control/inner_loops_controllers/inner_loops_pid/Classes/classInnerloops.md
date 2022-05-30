---
title: Innerloops
summary: Implementation of the inner loops. Computes the forces and torques (tau) to be applied on the vehicle based on desired reference values. 

---

# Innerloops



Implementation of the inner loops. Computes the forces and torques (tau) to be applied on the vehicle based on desired reference values. 


`#include <innerloops.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[Innerloops](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classInnerloops/#function-innerloops)**(ros::NodeHandle & nh)<br>Contructor of the innerloops class.  |
| | **[~Innerloops](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classInnerloops/#function-~innerloops)**()<br>Destructor of the innerloops class.  |

## Public Functions Documentation

### function Innerloops

```cpp
Innerloops(
    ros::NodeHandle & nh
)
```

Contructor of the innerloops class. 

**Parameters**: 

  * **nh** ROS nodehandle to subscribe, publish and read parameters. 


### function ~Innerloops

```cpp
~Innerloops()
```

Destructor of the innerloops class. 

-------------------------------

Updated on 2022-05-30 at 08:04:28 +0000