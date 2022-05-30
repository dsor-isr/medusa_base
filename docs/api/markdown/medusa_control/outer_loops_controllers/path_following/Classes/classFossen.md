---
title: Fossen
summary: Path following using Fossen's algorithm for path following Method3: based on the work of Fossen(2015)

---

# Fossen



Path following using [Fossen]()'s algorithm for path following Method3: based on the work of [Fossen(2015)]() [More...](#detailed-description)


`#include <Fossen.h>`

Inherits from [PathFollowing](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[Fossen](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-fossen)**(ros::Publisher surge_pub, ros::Publisher yaw_pub, ros::ServiceClient mode_client)<br>Constructor method for the Path Following class.  |
| virtual bool | **[setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-setpfgains)**(std::vector< double > gains) override<br>Method that given an array of doubles, updates the gains of the controller.  |
| virtual void | **[callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-callpfcontroller)**(double dt) override<br>Method that implements the path following control law.  |
| virtual void | **[publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-publish-private)**() override<br>Method to publish the data from the path following.  |
| virtual void | **[start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-start)**() override<br>Method used to start the algorithm in the first run.  |
| virtual bool | **[stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-stop)**() override<br>Method used to check whether we reached the end of the algorithm or not.  |
| virtual bool | **[reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classFossen/#function-reset)**() override<br>Method used to reset the algorithm control parameters when running the algorithm more than once.  |

## Additional inherited members

**Public Functions inherited from [PathFollowing](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/)**

|                | Name           |
| -------------- | -------------- |
| virtual | **[~PathFollowing](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-~pathfollowing)**()<br>Virtual destructor for the abstract pathfollowing class.  |
| void | **[publish](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-publish)**()<br>Method to publish the data given by the algorithm.  |
| virtual bool | **[resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-resetvirtualtarget)**(float value)<br>Method to reset the virtual target of the vehicle (gamma) to a pre-specified value. Not all controllers need this (example: [Samson](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/), [Fossen]() which use the closest point)  |
| bool | **[resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-resetvirtualtarget)**()<br>Method to reset the virtual target of the vehicle (gamma) to zero. Not all controllers need this (example: [Samson](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/), [Fossen]() which use the closest point)  |
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
class Fossen;
```

Path following using [Fossen]()'s algorithm for path following Method3: based on the work of [Fossen(2015)]()

**Author**: 

  * [Marcelo](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/) Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 

This algorithm support: Controls:

* yaw
* surge Supports Cooperative Path Following - True Contains Currents Observers - False

## Public Functions Documentation

### function Fossen

```cpp
Fossen(
    ros::Publisher surge_pub,
    ros::Publisher yaw_pub,
    ros::ServiceClient mode_client
)
```

Constructor method for the Path Following class. 

**Parameters**: 

  * **surge_pub** The ROS surge publisher 
  * **yaw_pub** The ROS yaw publisher 
  * **mode_client** The ROS client for the path (to change the mode of operation to closest point) 


### function setPFGains

```cpp
virtual bool setPFGains(
    std::vector< double > gains
) override
```

Method that given an array of doubles, updates the gains of the controller. 

**Parameters**: 

  * **gains** The gains of the controller


**Return**: By default just returns false for this algorithm 

**Reimplements**: [PathFollowing::setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-setpfgains)



NOTE: In this controller this method does nothing, as there are no gains to tweak. They are all fixed


### function callPFController

```cpp
virtual void callPFController(
    double dt
) override
```

Method that implements the path following control law. 

**Parameters**: 

  * **dt** The time diference between last call and current call (in seconds)
  * **dt** The time difference in seconds 


**Reimplements**: [PathFollowing::callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-callpfcontroller)


Method that implements the path Following algorihtm.


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

Method used to start the algorithm in the first run. 

**Return**: the success of the operation 

**Reimplements**: [PathFollowing::start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-start)


### function stop

```cpp
virtual bool stop() override
```

Method used to check whether we reached the end of the algorithm or not. 

**Return**: the success of the operation 

**Reimplements**: [PathFollowing::stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-stop)


### function reset

```cpp
virtual bool reset() override
```

Method used to reset the algorithm control parameters when running the algorithm more than once. 

**Return**: Whether the reset was made successfully or not 

**Reimplements**: [PathFollowing::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-reset)


-------------------------------

Updated on 2022-05-30 at 08:04:30 +0000