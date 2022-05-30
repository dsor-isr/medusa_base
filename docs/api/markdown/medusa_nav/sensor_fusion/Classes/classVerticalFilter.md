---
title: VerticalFilter
summary: This Class estimates the state of the vehicle in the horizontal frame. The state includes depth, velocity, altitude, and bouyancy. The latest state estimate of the filter is obtained using the exposed function getEstimate(). Measurement updates to the filter is done either using measCallback(). 

---

# VerticalFilter



This Class estimates the state of the vehicle in the horizontal frame. The state includes depth, velocity, altitude, and bouyancy. The latest state estimate of the filter is obtained using the exposed function getEstimate(). Measurement updates to the filter is done either using measCallback().  [More...](#detailed-description)


`#include <VerticalFilter.h>`

## Public Classes

|                | Name           |
| -------------- | -------------- |
| struct | **[config](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/)**  |

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[VerticalFilter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classVerticalFilter/#function-verticalfilter)**()<br>Constructor.  |
| virtual | **[~VerticalFilter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classVerticalFilter/#function-~verticalfilter)**() =default<br>Destructor.  |
| void | **[configure](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classVerticalFilter/#function-configure)**(const [VerticalFilter::config](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/) configurations)<br>Configure filter program variables and set initialization conditions.  |
| void | **[computePredict](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classVerticalFilter/#function-computepredict)**(auv_msgs::NavigationStatus & state, ros::Time t_request)<br>Propagate the state to the current time.  |
| void | **[newMeasurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classVerticalFilter/#function-newmeasurement)**(const [FilterGimmicks::measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/) & m)<br>Checks and processes a new measurement.  |
| void | **[resetFilter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classVerticalFilter/#function-resetfilter)**()<br>Resets the filter, will require re-initialization to start back update again.  |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| const int | **[MEAS_LEN](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classVerticalFilter/#variable-meas-len)** <br>Measurement length.  |

## Detailed Description

```cpp
class VerticalFilter;
```

This Class estimates the state of the vehicle in the horizontal frame. The state includes depth, velocity, altitude, and bouyancy. The latest state estimate of the filter is obtained using the exposed function getEstimate(). Measurement updates to the filter is done either using measCallback(). 

@Note Bouyancy is estimated but the output is not exposed to the user. 

## Public Functions Documentation

### function VerticalFilter

```cpp
VerticalFilter()
```

Constructor. 

### function ~VerticalFilter

```cpp
virtual ~VerticalFilter() =default
```

Destructor. 

### function configure

```cpp
void configure(
    const VerticalFilter::config configurations
)
```

Configure filter program variables and set initialization conditions. 

**Parameters**: 

  * **config_** structure with configurations 


### function computePredict

```cpp
void computePredict(
    auv_msgs::NavigationStatus & state,
    ros::Time t_request
)
```

Propagate the state to the current time. 

**Parameters**: 

  * **state** State vector 
  * **t_request** current time 


### function newMeasurement

```cpp
void newMeasurement(
    const FilterGimmicks::measurement & m
)
```

Checks and processes a new measurement. 

**Parameters**: 

  * **m** Measurement


**Return**: true if measurement is processed successfully 

### function resetFilter

```cpp
void resetFilter()
```

Resets the filter, will require re-initialization to start back update again. 

## Public Attributes Documentation

### variable MEAS_LEN

```cpp
static const int MEAS_LEN = 3;
```

Measurement length. 

-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000