---
title: PathState
summary: A structure to hold the data of the path. 

---

# PathState



A structure to hold the data of the path.  [More...](#detailed-description)


`#include <States.h>`

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| double | **[gamma](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-gamma)** <br>The value of gamma used in the computations for this values.  |
| Eigen::Vector3d | **[pd](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-pd)** <br>Desired position and its derivatives.  |
| Eigen::Vector3d | **[d_pd](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-d-pd)**  |
| Eigen::Vector3d | **[dd_pd](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-dd-pd)**  |
| double | **[psi](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-psi)** <br>Other properties of the path such as the psi, curvature and tangent_norm.  |
| double | **[curvature](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-curvature)**  |
| double | **[tangent_norm](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-tangent-norm)**  |
| double | **[vd](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-vd)** <br>The desired speed for a given gamma.  |
| double | **[d_vd](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-d-vd)**  |
| double | **[vehicle_speed](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-vehicle-speed)**  |
| double | **[vc](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-vc)** <br>The correction speed for gamma (when cooperating with other vehicles.  |
| double | **[gamma_min](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-gamma-min)** <br>Properties related to the boundaries of the parameterization of the path.  |
| double | **[gamma_max](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/structPathState/#variable-gamma-max)**  |

## Detailed Description

```cpp
struct PathState;
```

A structure to hold the data of the path. 

**Author**: 

  * [Marcelo](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/path_following/Classes/classMarcelo/) Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 
## Public Attributes Documentation

### variable gamma

```cpp
double gamma {0.0};
```

The value of gamma used in the computations for this values. 

### variable pd

```cpp
Eigen::Vector3d pd {0.0, 0.0, 0.0};
```

Desired position and its derivatives. 

### variable d_pd

```cpp
Eigen::Vector3d d_pd {0.0, 0.0, 0.0};
```


### variable dd_pd

```cpp
Eigen::Vector3d dd_pd {0.0, 0.0, 0.0};
```


### variable psi

```cpp
double psi {0.0};
```

Other properties of the path such as the psi, curvature and tangent_norm. 

### variable curvature

```cpp
double curvature {0.0};
```


### variable tangent_norm

```cpp
double tangent_norm {0.0};
```


### variable vd

```cpp
double vd {0.0};
```

The desired speed for a given gamma. 

### variable d_vd

```cpp
double d_vd {0.0};
```


### variable vehicle_speed

```cpp
double vehicle_speed {0.0};
```


### variable vc

```cpp
double vc {0.0};
```

The correction speed for gamma (when cooperating with other vehicles. 

### variable gamma_min

```cpp
double gamma_min {std::numeric_limits<double>::lowest()};
```

Properties related to the boundaries of the parameterization of the path. 

### variable gamma_max

```cpp
double gamma_max {std::numeric_limits<double>::max()};
```


-------------------------------

Updated on 2022-05-30 at 08:04:30 +0000