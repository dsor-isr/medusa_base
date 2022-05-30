---
title: State
summary: State class - used to save the state of a vehicle (using SNAME convention) 

---

# State



[State]() class - used to save the state of a vehicle (using SNAME convention)  [More...](#detailed-description)


`#include <State.h>`

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| Eigen::Vector3d | **[eta1](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/structState/#variable-eta1)**  |
| Eigen::Vector3d | **[eta2](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/structState/#variable-eta2)**  |
| Eigen::Vector3d | **[v1](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/structState/#variable-v1)**  |
| Eigen::Vector3d | **[v2](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/structState/#variable-v2)**  |

## Detailed Description

```cpp
struct State;
```

[State]() class - used to save the state of a vehicle (using SNAME convention) 

**Author**: Marcelo Jacinto 

**Version**: 1.0.0 

**Date**: 2021/11/12 

**Copyright**: MIT 
## Public Attributes Documentation

### variable eta1

```cpp
Eigen::Vector3d eta1 {0.0, 0.0, 0.0};
```


The position of the vehicle expressed in the inertial frame eta1=[x,y,z]^T 


### variable eta2

```cpp
Eigen::Vector3d eta2 {0.0, 0.0, 0.0};
```


The orientation of the vehicle expressed in the inertial frame, using euler angles eta2=[roll, pitch, yaw]^T 


### variable v1

```cpp
Eigen::Vector3d v1 {0.0, 0.0, 0.0};
```


The body velocity of the vehicle v1=[u,v,w]^T 


### variable v2

```cpp
Eigen::Vector3d v2 {0.0, 0.0, 0.0};
```


The body angular velocity of the vehicle v2=[p,q,r]^T 


-------------------------------

Updated on 2022-05-30 at 08:04:25 +0000