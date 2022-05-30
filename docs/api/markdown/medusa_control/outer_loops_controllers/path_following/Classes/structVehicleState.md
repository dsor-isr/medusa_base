---
title: VehicleState
summary: A structure to hold the state of the vehicle. 

---

# VehicleState



A structure to hold the state of the vehicle.  [More...](#detailed-description)


`#include <States.h>`

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| Eigen::Vector3d | **[v1](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structVehicleState/#variable-v1)** <br>Velocities in the body frame.  |
| Eigen::Vector3d | **[v2](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structVehicleState/#variable-v2)**  |
| Eigen::Vector3d | **[eta1](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structVehicleState/#variable-eta1)** <br>Positions and orientations in inertial frame.  |
| Eigen::Vector3d | **[eta2](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structVehicleState/#variable-eta2)**  |

## Detailed Description

```cpp
struct VehicleState;
```

A structure to hold the state of the vehicle. 

**Author**: 

  * [Marcelo](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/) Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 
## Public Attributes Documentation

### variable v1

```cpp
Eigen::Vector3d v1 {0.0, 0.0, 0.0};
```

Velocities in the body frame. 

### variable v2

```cpp
Eigen::Vector3d v2 {0.0, 0.0, 0.0};
```


### variable eta1

```cpp
Eigen::Vector3d eta1 {0.0, 0.0, 0.0};
```

Positions and orientations in inertial frame. 

### variable eta2

```cpp
Eigen::Vector3d eta2 {0.0, 0.0, 0.0};
```


-------------------------------

Updated on 2022-05-30 at 08:04:30 +0000