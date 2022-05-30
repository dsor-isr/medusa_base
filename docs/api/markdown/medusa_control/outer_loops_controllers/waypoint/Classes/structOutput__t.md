---
title: Output_t
summary: Waypoint output struct. Contains orientation and linear and angular velocities. Not everything needs to be used. 

---

# Output_t



Waypoint output struct. Contains orientation and linear and angular velocities. Not everything needs to be used. 


`#include <wp_controller.h>`

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| Eigen::Vector3d | **[eta2](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/structOutput__t/#variable-eta2)**  |
| Eigen::Vector3d | **[v1](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/structOutput__t/#variable-v1)**  |
| Eigen::Vector3d | **[v2](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/structOutput__t/#variable-v2)**  |

## Public Attributes Documentation

### variable eta2

```cpp
Eigen::Vector3d eta2 {0.0, 0.0, 0.0};
```


### variable v1

```cpp
Eigen::Vector3d v1 {0.0, 0.0, 0.0};
```


### variable v2

```cpp
Eigen::Vector3d v2 {0.0, 0.0, 0.0};
```


-------------------------------

Updated on 2022-05-30 at 08:04:30 +0000