---
title: RotationalFilter
summary: This Class estimates the state of the vehicle in the rotational frame. The state includes orientation and angular velocity. The latest state estimate of the filter is obtained using the function computePredict(). Measurement updates to the filter is done using newMeasurement(). 

---

# RotationalFilter



This Class estimates the state of the vehicle in the rotational frame. The state includes orientation and angular velocity. The latest state estimate of the filter is obtained using the function [computePredict()](). Measurement updates to the filter is done using [newMeasurement()](). 


`#include <RotationalFilter.h>`

## Public Classes

|                | Name           |
| -------------- | -------------- |
| struct | **[config](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structRotationalFilter_1_1config/)**  |

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[RotationalFilter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classRotationalFilter/#function-rotationalfilter)**()<br>Rotational Filter Constructor.  |
| virtual | **[~RotationalFilter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classRotationalFilter/#function-~rotationalfilter)**() =default<br>Rotational Filter Destructor.  |
| void | **[computePredict](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classRotationalFilter/#function-computepredict)**(auv_msgs::NavigationStatus & state, const ros::Time & t_request)<br>Progate the state to the time t_request.  |
| void | **[configure](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classRotationalFilter/#function-configure)**(const [config](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structRotationalFilter_1_1config/) configurations)<br>configure filter program variables and may initializes the filter  |
| void | **[newMeasurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classRotationalFilter/#function-newmeasurement)**(const [FilterGimmicks::measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/) & m)<br>Checks and processes a new measurement.  |
| void | **[resetFilter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classRotationalFilter/#function-resetfilter)**()<br>Reset the horizontal filter.  |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| const int | **[STATE_LEN](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classRotationalFilter/#variable-state-len)**  |
| double | **[PI](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classRotationalFilter/#variable-pi)**  |

## Public Functions Documentation

### function RotationalFilter

```cpp
RotationalFilter()
```

Rotational Filter Constructor. 

### function ~RotationalFilter

```cpp
virtual ~RotationalFilter() =default
```

Rotational Filter Destructor. 

### function computePredict

```cpp
void computePredict(
    auv_msgs::NavigationStatus & state,
    const ros::Time & t_request
)
```

Progate the state to the time t_request. 

**Parameters**: 

  * **state** 
  * **t_request** 


**Note**: Why have we used TF here? We want: Rotation of base_link wrt world, i.e. header = world, child = base_link (TFwb). I have: 1. static_tf - Rotation of imu wrt base_link, i.e. header = base_link, child = imu (TFbi); 2. sensor output - Rotation of imu wrt world, i.e. header = world, child = imu (TFwi). Solution: TFwb = TFwi * (TFbi)^-1 

### function configure

```cpp
void configure(
    const config configurations
)
```

configure filter program variables and may initializes the filter 

**Parameters**: 

  * **configurations** struct to store configurations from yaml file 


### function newMeasurement

```cpp
void newMeasurement(
    const FilterGimmicks::measurement & m
)
```

Checks and processes a new measurement. 

**Parameters**: 

  * **m** measurement 


### function resetFilter

```cpp
void resetFilter()
```

Reset the horizontal filter. 

## Public Attributes Documentation

### variable STATE_LEN

```cpp
static const int STATE_LEN = 6;
```


### variable PI

```cpp
double PI = 3.1415926;
```


-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000