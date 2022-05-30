---
title: Lapierre
summary: Path following using Lapierre's algorithm for path following Method2: developed from the work of Lionel Lapierre and Antonio(2003) 

---

# Lapierre



Path following using [Lapierre]()'s algorithm for path following Method2: developed from the work of Lionel [Lapierre]() and Antonio(2003)  [More...](#detailed-description)


`#include <Lapierre.h>`

Inherits from [PathFollowing](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[Lapierre](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-lapierre)**(double k1, double k2, double k3, double theta, double k_delta, ros::Publisher surge_pub, ros::Publisher yaw_rate_pub, ros::Publisher rabbit_pub)<br>Constructor method for the Path Following class.  |
| virtual bool | **[setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-setpfgains)**(std::vector< double > gains) override<br>Method that given a vector of doubles, updates the gains of the controller.  |
| virtual void | **[callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-callpfcontroller)**(double dt) override<br>Method that implements the path following control law.  |
| virtual void | **[publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-publish-private)**() override<br>Method to publish the data from the path following.  |
| virtual void | **[start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-start)**() override<br>Method to run in the first iteration of the path following algorithm.  |
| virtual bool | **[stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-stop)**() override<br>Method used to check whether we reached the end of the algorithm or not.  |
| virtual bool | **[reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-reset)**() override<br>Method used to reset the algorithm control parameters when running the algorithm more than once.  |
| virtual bool | **[resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classLapierre/#function-resetvirtualtarget)**(float value) override<br>Method to reset the virtual target of the vehicle (gamma) to a pre-specified value. Not all controllers need this (example: [Samson](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/), [Fossen]() which use the closest point)  |

## Additional inherited members

**Public Functions inherited from [PathFollowing](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/)**

|                | Name           |
| -------------- | -------------- |
| virtual | **[~PathFollowing](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-~pathfollowing)**()<br>Virtual destructor for the abstract pathfollowing class.  |
| void | **[publish](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-publish)**()<br>Method to publish the data given by the algorithm.  |
| void | **[UpdateVehicleState](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-updatevehiclestate)**(const [VehicleState](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structVehicleState/) & vehicle_state)<br>Method to update the vehicle state used by the controller.  |
| void | **[UpdatePathState](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-updatepathstate)**(const [PathState](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/) & path_state)<br>Method to update the path state used by the controller.  |
| void | **[setPFollowingDebugPublisher](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-setpfollowingdebugpublisher)**(const ros::Publisher & pfollowing_debug_pub)<br>Method to set common publishers.  |

**Protected Functions inherited from [PathFollowing](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/)**

|                | Name           |
| -------------- | -------------- |
| double | **[algConvert](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-algconvert)**(double alg_new, double alg_old, double alg_out_old)<br>Auxiliar method to smooth out the angle to be used by path following algorithms.  |

**Protected Attributes inherited from [PathFollowing](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/)**

|                | Name           |
| -------------- | -------------- |
| [VehicleState](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structVehicleState/) | **[vehicle_state_](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#variable-vehicle-state-)** <br>Variable to store the state of the vehicle.  |
| [PathState](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/) | **[path_state_](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#variable-path-state-)** <br>Variable to store the state of the path.  |
| [PFollowingDebug](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPFollowingDebug/) | **[pfollowing_debug_](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#variable-pfollowing-debug-)** <br>Variable to store the state of the path.  |
| ros::Publisher | **[pfollowing_debug_pub_](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#variable-pfollowing-debug-pub-)**  |


## Detailed Description

```cpp
class Lapierre;
```

Path following using [Lapierre]()'s algorithm for path following Method2: developed from the work of Lionel [Lapierre]() and Antonio(2003) 

**Author**: 

  * [Marcelo](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/) Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 

This algorithm support: Controls:

* yaw-rate
* surge
* virtual-target (gamma) Supports Cooperative Path Following - True Contains Currents Observers - False

## Public Functions Documentation

### function Lapierre

```cpp
Lapierre(
    double k1,
    double k2,
    double k3,
    double theta,
    double k_delta,
    ros::Publisher surge_pub,
    ros::Publisher yaw_rate_pub,
    ros::Publisher rabbit_pub
)
```

Constructor method for the Path Following class. 

**Parameters**: 

  * **k1** Controller gain 
  * **k2** Controller gain 
  * **k3** Controller gain 
  * **theta** Controller gain 
  * **k_delta** Controller gain 
  * **surge_pub** The ROS surge publisher 
  * **yaw_rate_pub** The ROS yaw rate publisher 
  * **rabbit_pub** The ROS rabbit publisher 


### function setPFGains

```cpp
virtual bool setPFGains(
    std::vector< double > gains
) override
```

Method that given a vector of doubles, updates the gains of the controller. 

**Parameters**: 

  * **gains** A vector of gains


**Return**: a boolean which represents the success of the operation 

**Reimplements**: [PathFollowing::setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-setpfgains)



NOTE: The default order of the gains is k1, k2, k3, theta, k_delta


### function callPFController

```cpp
virtual void callPFController(
    double dt
) override
```

Method that implements the path following control law. 

**Parameters**: 

  * **dt** The time diference between the last and current call (in seconds) 


**Reimplements**: [PathFollowing::callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-callpfcontroller)


### function publish_private

```cpp
virtual void publish_private() override
```

Method to publish the data from the path following. 

**Reimplements**: [PathFollowing::publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-publish-private)


### function start

```cpp
virtual void start() override
```

Method to run in the first iteration of the path following algorithm. 

**Reimplements**: [PathFollowing::start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-start)


### function stop

```cpp
virtual bool stop() override
```

Method used to check whether we reached the end of the algorithm or not. 

**Return**: the success of the operation 

**Reimplements**: [PathFollowing::stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-stop)


Check if the gamma is greater then the gamma max of the path If so, we have reached the end


### function reset

```cpp
virtual bool reset() override
```

Method used to reset the algorithm control parameters when running the algorithm more than once. 

**Return**: Whether the reset was made successfully or not 

**Reimplements**: [PathFollowing::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-reset)


### function resetVirtualTarget

```cpp
virtual bool resetVirtualTarget(
    float value
) override
```

Method to reset the virtual target of the vehicle (gamma) to a pre-specified value. Not all controllers need this (example: [Samson](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/), [Fossen]() which use the closest point) 

**Return**: Whether the reset was made successfully or not 

**Reimplements**: [PathFollowing::resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-resetvirtualtarget)


-------------------------------

Updated on 2022-05-30 at 08:04:30 +0000