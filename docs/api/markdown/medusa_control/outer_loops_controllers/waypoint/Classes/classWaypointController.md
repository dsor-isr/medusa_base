---
title: WaypointController
summary: Abstract class of a waypoint controller. 

---

# WaypointController



Abstract class of a waypoint controller. 


`#include <wp_controller.h>`

Inherited by [WpHeading](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpHeading/), [WpLoose](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpLoose/), [WpStandard](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpStandard/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[WaypointController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-waypointcontroller)**() |
| virtual | **[~WaypointController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-~waypointcontroller)**() |
| void | **[setFrequency](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-setfrequency)**(const double & f)<br>Mutator for setting the controller period.  |
| void | **[setGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-setgains)**(const std::vector< double > & gains)<br>Mutator for updating the gains.  |
| void | **[compute](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-compute)**([Vehicle_t](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/structVehicle__t/) state, [WPref_t](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/structWPref__t/) wp_ref)<br>Computes and publishes the output of the controller.  |

## Protected Functions

|                | Name           |
| -------------- | -------------- |
| virtual void | **[publish](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-publish)**() =0<br>Virtual function to publish the output of the controller.  |
| virtual void | **[calculateRef](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-calculateref)**([Vehicle_t](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/structVehicle__t/) state, [WPref_t](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/structWPref__t/) wp_ref) =0<br>Virtual function to compute the waypoint controller.  |
| double | **[getYawOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-getyawout)**()<br>Getter of the yaw reference (output of controller)  |
| double | **[getYawrateOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-getyawrateout)**()<br>Getter of the yaw rate reference (output of controller)  |
| double | **[getSurgeOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-getsurgeout)**()<br>Getter of the surge reference (output of controller)  |
| double | **[getSwayOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-getswayout)**()<br>Getter of the sway reference (output of controller)  |
| void | **[setYawOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-setyawout)**(const double & value)<br>Yaw reference setter (output of controller)  |
| void | **[setYawrateOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-setyawrateout)**(const double & value)<br>Yaw rate reference setter (output of controller)  |
| void | **[setSurgeOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-setsurgeout)**(const double & value)<br>Surge reference setter (output of controller)  |
| void | **[setSwayOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-setswayout)**(const double & value)<br>Sway reference setter (output of controller)  |

## Protected Attributes

|                | Name           |
| -------------- | -------------- |
| double | **[ts_](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#variable-ts-)**  |
| std::vector< double > | **[gains_](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#variable-gains-)**  |

## Public Functions Documentation

### function WaypointController

```cpp
inline WaypointController()
```


### function ~WaypointController

```cpp
inline virtual ~WaypointController()
```


### function setFrequency

```cpp
inline void setFrequency(
    const double & f
)
```

Mutator for setting the controller period. 

**Parameters**: 

  * **f** frequency of the main loop 


### function setGains

```cpp
inline void setGains(
    const std::vector< double > & gains
)
```

Mutator for updating the gains. 

**Parameters**: 

  * **gains** parameters of the waypoiny controller 


### function compute

```cpp
inline void compute(
    Vehicle_t state,
    WPref_t wp_ref
)
```

Computes and publishes the output of the controller. 

**Parameters**: 

  * **state** 
  * **wp_ref** 


## Protected Functions Documentation

### function publish

```cpp
virtual void publish() =0
```

Virtual function to publish the output of the controller. 

**Reimplemented by**: [WpLoose::publish](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpLoose/#function-publish), [WpHeading::publish](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpHeading/#function-publish), [WpStandard::publish](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpStandard/#function-publish)


### function calculateRef

```cpp
virtual void calculateRef(
    Vehicle_t state,
    WPref_t wp_ref
) =0
```

Virtual function to compute the waypoint controller. 

**Parameters**: 

  * **state** 
  * **wp_ref** 


**Reimplemented by**: [WpLoose::calculateRef](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpLoose/#function-calculateref), [WpHeading::calculateRef](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpHeading/#function-calculateref), [WpStandard::calculateRef](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpStandard/#function-calculateref)


### function getYawOut

```cpp
inline double getYawOut()
```

Getter of the yaw reference (output of controller) 

**Return**: yaw ref 

### function getYawrateOut

```cpp
inline double getYawrateOut()
```

Getter of the yaw rate reference (output of controller) 

**Return**: yaw rate ref 

### function getSurgeOut

```cpp
inline double getSurgeOut()
```

Getter of the surge reference (output of controller) 

**Return**: surge ref 

### function getSwayOut

```cpp
inline double getSwayOut()
```

Getter of the sway reference (output of controller) 

**Return**: sway ref 

### function setYawOut

```cpp
inline void setYawOut(
    const double & value
)
```

Yaw reference setter (output of controller) 

**Parameters**: 

  * **value** yaw ref 


### function setYawrateOut

```cpp
inline void setYawrateOut(
    const double & value
)
```

Yaw rate reference setter (output of controller) 

**Parameters**: 

  * **value** yaw rate ref 


### function setSurgeOut

```cpp
inline void setSurgeOut(
    const double & value
)
```

Surge reference setter (output of controller) 

**Parameters**: 

  * **value** surge ref 


### function setSwayOut

```cpp
inline void setSwayOut(
    const double & value
)
```

Sway reference setter (output of controller) 

**Parameters**: 

  * **value** sway ref 


## Protected Attributes Documentation

### variable ts_

```cpp
double ts_ {0.0};
```


### variable gains_

```cpp
std::vector< double > gains_;
```


-------------------------------

Updated on 2022-05-30 at 08:04:30 +0000