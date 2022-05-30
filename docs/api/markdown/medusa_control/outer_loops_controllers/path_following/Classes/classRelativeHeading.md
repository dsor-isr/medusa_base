---
title: RelativeHeading
summary: Path following using RelativeHeading's algorithm for path following*. 

---

# RelativeHeading



Path following using [RelativeHeading]()'s algorithm for path following*.  [More...](#detailed-description)


`#include <RelativeHeading.h>`

Inherits from [PathFollowing](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[RelativeHeading](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-relativeheading)**(double kx, double ky, double kz, const Eigen::Vector2d & p_sat, double yaw_offset, ros::Publisher surge_pub, ros::Publisher sway_pub, ros::Publisher yaw_pub, ros::Publisher rabbit_pub)<br>Constructor method for the Path Following class.  |
| virtual bool | **[setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-setpfgains)**(std::vector< double > gains) override<br>Method to update the gains, given a vector of doubles.  |
| virtual void | **[callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-callpfcontroller)**(double dt) override<br>Method that implements the path following control law.  |
| virtual void | **[publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-publish-private)**() override<br>Method to publish_private the data from the path following.  |
| virtual void | **[start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-start)**() override<br>Method used in the first run to do initial setup.  |
| virtual bool | **[stop](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-stop)**() override<br>Method used to check whether we reached the end of the algorithm or not.  |
| virtual bool | **[reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-reset)**() override<br>Method used to reset the algorithm control parameters when running the algorithm more than once.  |
| virtual bool | **[resetVirtualTarget](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classRelativeHeading/#function-resetvirtualtarget)**(float value) override<br>Method to reset the virtual target of the vehicle (gamma) to a pre-specified value. Not all controllers need this (example: [Samson](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classSamson/), [Fossen]() which use the closest point)  |

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
class RelativeHeading;
```

Path following using [RelativeHeading]()'s algorithm for path following*. 

**Author**: 

  * Francisco Rego 
  * Andre Potes 
  * [Marcelo](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/) Jacinto 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 

This algorithm support: Controls:

* yaw
* surge
* sway Supports Cooperative Path Following - True Contains Currents Observers - False

## Public Functions Documentation

### function RelativeHeading

```cpp
RelativeHeading(
    double kx,
    double ky,
    double kz,
    const Eigen::Vector2d & p_sat,
    double yaw_offset,
    ros::Publisher surge_pub,
    ros::Publisher sway_pub,
    ros::Publisher yaw_pub,
    ros::Publisher rabbit_pub
)
```

Constructor method for the Path Following class. 

**Parameters**: 

  * **surge_pub** The ROS surge publisher 
  * **sway_pub** The ROS sway publisher 
  * **yaw_pub** The ROS yaw publisher 
  * **rabbit_pub** The ROS rabbit publisher 


### function setPFGains

```cpp
virtual bool setPFGains(
    std::vector< double > gains
) override
```

Method to update the gains, given a vector of doubles. 

**Parameters**: 

  * **gains** A vector of gains to update in the controller The default order is: TODO


**Return**: a boolean which represents the success of the operation 

**Reimplements**: [PathFollowing::setPFGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-setpfgains)


### function callPFController

```cpp
virtual void callPFController(
    double dt
) override
```

Method that implements the path following control law. 

**Parameters**: 

  * **dt** The time step between last call and current call (in seconds) 


**Reimplements**: [PathFollowing::callPFController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-callpfcontroller)


### function publish_private

```cpp
virtual void publish_private() override
```

Method to publish_private the data from the path following. 

**Reimplements**: [PathFollowing::publish_private](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-publish-private)


### function start

```cpp
virtual void start() override
```

Method used in the first run to do initial setup. 

**Reimplements**: [PathFollowing::start](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classPathFollowing/#function-start)


### function stop

```cpp
virtual bool stop() override
```

Method used to check whether we reached the end of the algorithm or not. 

**Return**: The success of the operation 

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