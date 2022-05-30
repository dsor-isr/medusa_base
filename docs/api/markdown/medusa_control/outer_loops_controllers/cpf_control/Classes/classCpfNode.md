---
title: CpfNode
summary: ROS node to actually do the Cooperative Control. 

---

# CpfNode



ROS node to actually do the Cooperative Control.  [More...](#detailed-description)


`#include <CpfNode.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[CpfNode](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCpfNode/#function-cpfnode)**(ros::NodeHandle * nh, ros::NodeHandle * nh_p)<br>The constructor for the Cooperative control law.  |
| | **[~CpfNode](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCpfNode/#function-~cpfnode)**()<br>Destructor for the CPF Node. Here all the subscribers, publishers and timers are shutdown.  |

## Detailed Description

```cpp
class CpfNode;
```

ROS node to actually do the Cooperative Control. 

**Author**: 

  * Marcelo Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 
## Public Functions Documentation

### function CpfNode

```cpp
CpfNode(
    ros::NodeHandle * nh,
    ros::NodeHandle * nh_p
)
```

The constructor for the Cooperative control law. 

**Parameters**: 

  * **nh** The public nodehandle 
  * **nh_p** The private nodehandle
  * **nh** Pointer to the public nodehandle 
  * **nh** Pointer to the private nodehandle 


Constructor for the [CpfNode](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCpfNode/).


### function ~CpfNode

```cpp
~CpfNode()
```

Destructor for the CPF Node. Here all the subscribers, publishers and timers are shutdown. 

Destructor for the [CpfNode](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCpfNode/). 


-------------------------------

Updated on 2022-05-30 at 08:04:27 +0000