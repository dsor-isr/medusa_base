---
title: PathFollowing
summary: A Base class to update the path following law. 

---

# PathFollowing



A Base class to update the path following law.  [More...](#detailed-description)


`#include <PathFollowing.h>`

Inherited by [Aguiar](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classAguiar/), [Brevik](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classBrevik/), [Fossen](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/), [Lapierre](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/), [Marcelo](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/), [Pramod](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPramod/), [RelativeHeading](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/), [Romulo](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRomulo/), [Samson](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| virtual | **[~PathFollowing](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-~pathfollowing)**()<br>Virtual destructor for the abstract pathfollowing class.  |
| virtual void | **[callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-callpfcontroller)**(double dt) =0<br>Method to update the path following control law.  |
| void | **[publish](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-publish)**()<br>Method to publish the data given by the algorithm.  |
| virtual void | **[publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-publish-private)**() =0 |
| virtual void | **[start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-start)**() =0<br>Method used to setup the algorithm in the first iteration.  |
| virtual bool | **[stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-stop)**() =0<br>Method used to check whether we have reached the end of the path following algorithm or not. This method will be called in every iteration of the algorithm, and when it return true, the algorithm will stop.  |
| virtual bool | **[reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-reset)**() =0<br>Method used to reset the algorithm control parameters when running the algorithm more than once.  |
| virtual bool | **[resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-resetvirtualtarget)**(float value)<br>Method to reset the virtual target of the vehicle (gamma) to a pre-specified value. Not all controllers need this (example: [Samson](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/), [Fossen]() which use the closest point)  |
| bool | **[resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-resetvirtualtarget)**()<br>Method to reset the virtual target of the vehicle (gamma) to zero. Not all controllers need this (example: [Samson](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/), [Fossen]() which use the closest point)  |
| virtual bool | **[setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-setpfgains)**(std::vector< double > gains) =0<br>Receives a vector of gains that should be mapped to the specific controller gains.  |
| void | **[UpdateVehicleState](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-updatevehiclestate)**(const [VehicleState](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structVehicleState/) & vehicle_state)<br>Method to update the vehicle state used by the controller.  |
| void | **[UpdatePathState](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-updatepathstate)**(const [PathState](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/) & path_state)<br>Method to update the path state used by the controller.  |
| void | **[setPFollowingDebugPublisher](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-setpfollowingdebugpublisher)**(const ros::Publisher & pfollowing_debug_pub)<br>Method to set common publishers.  |

## Protected Functions

|                | Name           |
| -------------- | -------------- |
| double | **[algConvert](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-algconvert)**(double alg_new, double alg_old, double alg_out_old)<br>Auxiliar method to smooth out the angle to be used by path following algorithms.  |

## Protected Attributes

|                | Name           |
| -------------- | -------------- |
| [VehicleState](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structVehicleState/) | **[vehicle_state_](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#variable-vehicle-state-)** <br>Variable to store the state of the vehicle.  |
| [PathState](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/) | **[path_state_](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#variable-path-state-)** <br>Variable to store the state of the path.  |
| [PFollowingDebug](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPFollowingDebug/) | **[pfollowing_debug_](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#variable-pfollowing-debug-)** <br>Variable to store the state of the path.  |
| ros::Publisher | **[pfollowing_debug_pub_](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#variable-pfollowing-debug-pub-)**  |

## Detailed Description

```cpp
class PathFollowing;
```

A Base class to update the path following law. 

**Author**: 

  * [Marcelo](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/) Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 
## Public Functions Documentation

### function ~PathFollowing

```cpp
virtual ~PathFollowing()
```

Virtual destructor for the abstract pathfollowing class. 

### function callPFController

```cpp
virtual void callPFController(
    double dt
) =0
```

Method to update the path following control law. 

**Parameters**: 

  * **dt** The time diference between the current and previous call (in seconds) 


**Reimplemented by**: [Pramod::callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPramod/#function-callpfcontroller), [RelativeHeading::callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-callpfcontroller), [Romulo::callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRomulo/#function-callpfcontroller), [Fossen::callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-callpfcontroller), [Brevik::callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classBrevik/#function-callpfcontroller), [Marcelo::callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/#function-callpfcontroller), [Samson::callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/#function-callpfcontroller), [Lapierre::callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-callpfcontroller), [Aguiar::callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classAguiar/#function-callpfcontroller)


### function publish

```cpp
void publish()
```

Method to publish the data given by the algorithm. 

### function publish_private

```cpp
virtual void publish_private() =0
```


**Reimplemented by**: [Pramod::publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPramod/#function-publish-private), [RelativeHeading::publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-publish-private), [Romulo::publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRomulo/#function-publish-private), [Fossen::publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-publish-private), [Brevik::publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classBrevik/#function-publish-private), [Marcelo::publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/#function-publish-private), [Samson::publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/#function-publish-private), [Lapierre::publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-publish-private), [Aguiar::publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classAguiar/#function-publish-private)


### function start

```cpp
virtual void start() =0
```

Method used to setup the algorithm in the first iteration. 

**Reimplemented by**: [Pramod::start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPramod/#function-start), [RelativeHeading::start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-start), [Romulo::start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRomulo/#function-start), [Brevik::start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classBrevik/#function-start), [Marcelo::start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/#function-start), [Samson::start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/#function-start), [Fossen::start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-start), [Lapierre::start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-start), [Aguiar::start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classAguiar/#function-start)


### function stop

```cpp
virtual bool stop() =0
```

Method used to check whether we have reached the end of the path following algorithm or not. This method will be called in every iteration of the algorithm, and when it return true, the algorithm will stop. 

**Return**: A boolean that represents whether we have reached the end 

**Reimplemented by**: [Pramod::stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPramod/#function-stop), [RelativeHeading::stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-stop), [Romulo::stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRomulo/#function-stop), [Brevik::stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classBrevik/#function-stop), [Marcelo::stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/#function-stop), [Samson::stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/#function-stop), [Fossen::stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-stop), [Lapierre::stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-stop), [Aguiar::stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classAguiar/#function-stop)


### function reset

```cpp
virtual bool reset() =0
```

Method used to reset the algorithm control parameters when running the algorithm more than once. 

**Return**: Whether the reset was made successfully or not 

**Reimplemented by**: [Pramod::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPramod/#function-reset), [RelativeHeading::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-reset), [Romulo::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRomulo/#function-reset), [Brevik::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classBrevik/#function-reset), [Marcelo::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/#function-reset), [Samson::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/#function-reset), [Fossen::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-reset), [Lapierre::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-reset), [Aguiar::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classAguiar/#function-reset)


### function resetVirtualTarget

```cpp
virtual bool resetVirtualTarget(
    float value
)
```

Method to reset the virtual target of the vehicle (gamma) to a pre-specified value. Not all controllers need this (example: [Samson](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/), [Fossen]() which use the closest point) 

**Return**: Whether the reset was made successfully or not 

**Reimplemented by**: [RelativeHeading::resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-resetvirtualtarget), [Romulo::resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRomulo/#function-resetvirtualtarget), [Brevik::resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classBrevik/#function-resetvirtualtarget), [Marcelo::resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/#function-resetvirtualtarget), [Lapierre::resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-resetvirtualtarget), [Aguiar::resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classAguiar/#function-resetvirtualtarget)


### function resetVirtualTarget

```cpp
bool resetVirtualTarget()
```

Method to reset the virtual target of the vehicle (gamma) to zero. Not all controllers need this (example: [Samson](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/), [Fossen]() which use the closest point) 

**Return**: Whether the reset was made successfully or not 

This method calls the [resetVirtualTarget(float value)](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-resetvirtualtarget) method which can be overriden by each pf controller 


### function setPFGains

```cpp
virtual bool setPFGains(
    std::vector< double > gains
) =0
```

Receives a vector of gains that should be mapped to the specific controller gains. 

**Parameters**: 

  * **gains** A vector of gains for the controller 


**Reimplemented by**: [Pramod::setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPramod/#function-setpfgains), [RelativeHeading::setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-setpfgains), [Romulo::setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRomulo/#function-setpfgains), [Fossen::setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-setpfgains), [Brevik::setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classBrevik/#function-setpfgains), [Marcelo::setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/#function-setpfgains), [Samson::setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/#function-setpfgains), [Lapierre::setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-setpfgains), [Aguiar::setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classAguiar/#function-setpfgains)


This method must be implemented by each Path Following class


### function UpdateVehicleState

```cpp
void UpdateVehicleState(
    const VehicleState & vehicle_state
)
```

Method to update the vehicle state used by the controller. 

**Parameters**: 

  * **vehicle_state** A structure with the current state of the vehicle 


### function UpdatePathState

```cpp
void UpdatePathState(
    const PathState & path_state
)
```

Method to update the path state used by the controller. 

**Parameters**: 

  * **path_state** A structure with the current state of the path 


### function setPFollowingDebugPublisher

```cpp
inline void setPFollowingDebugPublisher(
    const ros::Publisher & pfollowing_debug_pub
)
```

Method to set common publishers. 

## Protected Functions Documentation

### function algConvert

```cpp
double algConvert(
    double alg_new,
    double alg_old,
    double alg_out_old
)
```

Auxiliar method to smooth out the angle to be used by path following algorithms. 

## Protected Attributes Documentation

### variable vehicle_state_

```cpp
VehicleState vehicle_state_;
```

Variable to store the state of the vehicle. 

### variable path_state_

```cpp
PathState path_state_;
```

Variable to store the state of the path. 

### variable pfollowing_debug_

```cpp
PFollowingDebug pfollowing_debug_;
```

Variable to store the state of the path. 

### variable pfollowing_debug_pub_

```cpp
ros::Publisher pfollowing_debug_pub_;
```


-------------------------------

Updated on 2022-05-30 at 08:04:30 +0000