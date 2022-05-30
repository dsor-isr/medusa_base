---
title: FiltersNode
summary: FiltersNode class. 

---

# FiltersNode



[FiltersNode]() class.  [More...](#detailed-description)


`#include <FiltersNode.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[FiltersNode](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classFiltersNode/#function-filtersnode)**(ros::NodeHandle * nh, ros::NodeHandle * nh_private)<br>Constructor.  |
| | **[~FiltersNode](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classFiltersNode/#function-~filtersnode)**()<br>Destructor.  |
| double | **[nodeFrequency](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classFiltersNode/#function-nodefrequency)**()<br>Defines the node frequency.  |

## Detailed Description

```cpp
class FiltersNode;
```

[FiltersNode]() class. 

**Note**: Read all the parameters for each filter(Horicontal, Rotations, Vertical), and configures them. Takes care of the iteration using timer. Also responsible for controlling the measurement buffer of the horizontal filter. 
## Public Functions Documentation

### function FiltersNode

```cpp
FiltersNode(
    ros::NodeHandle * nh,
    ros::NodeHandle * nh_private
)
```

Constructor. 

**Parameters**: 

  * **nh** node handle 
  * **nh_private** node handle private 


### function ~FiltersNode

```cpp
~FiltersNode()
```

Destructor. 

### function nodeFrequency

```cpp
double nodeFrequency()
```

Defines the node frequency. 

**Return**: Node frequency 

-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000