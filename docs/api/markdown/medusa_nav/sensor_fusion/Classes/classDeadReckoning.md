---
title: DeadReckoning
summary: DeadReckoning class. 

---

# DeadReckoning



[DeadReckoning]() class.  [More...](#detailed-description)


`#include <DeadReckoning.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[DeadReckoning](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classDeadReckoning/#function-deadreckoning)**(ros::NodeHandle * nh, ros::NodeHandle * nh_private)<br>Contructor [DeadReckoning](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classDeadReckoning/).  |
| | **[~DeadReckoning](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classDeadReckoning/#function-~deadreckoning)**()<br>Desctructor [DeadReckoning](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classDeadReckoning/).  |
| void | **[computePredict](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classDeadReckoning/#function-computepredict)**()<br>Propagate the state to the current time.  |

## Detailed Description

```cpp
class DeadReckoning;
```

[DeadReckoning]() class. 

**Note**: 

  * This node only outputs useful information when dvl is present 
  * Every time a new mission is deployed (FLAG=6), the DR is reseted to the current navigation state(x,y) 

## Public Functions Documentation

### function DeadReckoning

```cpp
DeadReckoning(
    ros::NodeHandle * nh,
    ros::NodeHandle * nh_private
)
```

Contructor [DeadReckoning](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classDeadReckoning/). 

### function ~DeadReckoning

```cpp
~DeadReckoning()
```

Desctructor [DeadReckoning](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classDeadReckoning/). 

### function computePredict

```cpp
void computePredict()
```

Propagate the state to the current time. 

**Return**: Success or Failure 

-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000