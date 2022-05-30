---
title: WpStandard
summary: Waypoint controller using surge and yaw, where the nose of the vehicle points to the desired position. 

---

# WpStandard



Waypoint controller using surge and yaw, where the nose of the vehicle points to the desired position. 


`#include <wp_standard.h>`

Inherits from [WaypointController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[WpStandard](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpStandard/#function-wpstandard)**(ros::Publisher surge_pub, ros::Publisher yaw_pub) |
| virtual | **[~WpStandard](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpStandard/#function-~wpstandard)**() |

## Additional inherited members

**Public Functions inherited from [WaypointController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/)**

|                | Name           |
| -------------- | -------------- |
| | **[WaypointController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-waypointcontroller)**() |
| virtual | **[~WaypointController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-~waypointcontroller)**() |
| void | **[setFrequency](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-setfrequency)**(const double & f)<br>Mutator for setting the controller period.  |
| void | **[setGains](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-setgains)**(const std::vector< double > & gains)<br>Mutator for updating the gains.  |
| void | **[compute](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-compute)**([Vehicle_t](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/structVehicle__t/) state, [WPref_t](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/structWPref__t/) wp_ref)<br>Computes and publishes the output of the controller.  |

**Protected Functions inherited from [WaypointController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/)**

|                | Name           |
| -------------- | -------------- |
| double | **[getYawOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-getyawout)**()<br>Getter of the yaw reference (output of controller)  |
| double | **[getYawrateOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-getyawrateout)**()<br>Getter of the yaw rate reference (output of controller)  |
| double | **[getSurgeOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-getsurgeout)**()<br>Getter of the surge reference (output of controller)  |
| double | **[getSwayOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-getswayout)**()<br>Getter of the sway reference (output of controller)  |
| void | **[setYawOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-setyawout)**(const double & value)<br>Yaw reference setter (output of controller)  |
| void | **[setYawrateOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-setyawrateout)**(const double & value)<br>Yaw rate reference setter (output of controller)  |
| void | **[setSurgeOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-setsurgeout)**(const double & value)<br>Surge reference setter (output of controller)  |
| void | **[setSwayOut](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#function-setswayout)**(const double & value)<br>Sway reference setter (output of controller)  |

**Protected Attributes inherited from [WaypointController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/)**

|                | Name           |
| -------------- | -------------- |
| double | **[ts_](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#variable-ts-)**  |
| std::vector< double > | **[gains_](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/#variable-gains-)**  |


## Public Functions Documentation

### function WpStandard

```cpp
WpStandard(
    ros::Publisher surge_pub,
    ros::Publisher yaw_pub
)
```


### function ~WpStandard

```cpp
inline virtual ~WpStandard()
```


-------------------------------

Updated on 2022-05-30 at 08:04:30 +0000