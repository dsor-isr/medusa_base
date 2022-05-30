---
title: HorizontalFilter
summary: Horizontal Filter class. 

---

# HorizontalFilter



Horizontal Filter class.  [More...](#detailed-description)


`#include <HorizontalFilter.h>`

## Public Classes

|                | Name           |
| -------------- | -------------- |
| struct | **[config](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structHorizontalFilter_1_1config/)**  |

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[HorizontalFilter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classHorizontalFilter/#function-horizontalfilter)**()<br>Contructor Horizontal Filter.  |
| virtual | **[~HorizontalFilter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classHorizontalFilter/#function-~horizontalfilter)**() =default<br>Desctructor Horizontal Filter.  |
| bool | **[computePredict](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classHorizontalFilter/#function-computepredict)**(auv_msgs::NavigationStatus & state, const ros::Time & t_request)<br>Propagate the state to the current time.  |
| void | **[configure](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classHorizontalFilter/#function-configure)**([HorizontalFilter::config](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structHorizontalFilter_1_1config/) & configurations)<br>Configure filter variables and set initialization conditions.  |
| void | **[newMeasurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classHorizontalFilter/#function-newmeasurement)**([FilterGimmicks::measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/) & m)<br>Tranforms the measurement from the sensor frame to the filter world frame. Calls addMeasurement and forwardPropagation methods.  |
| void | **[deleteMeasurementsInBuffer](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classHorizontalFilter/#function-deletemeasurementsinbuffer)**()<br>Clears all measurements older than a timer period defined in save_meas_interval.  |
| std::vector< double > | **[getExtimateCurrents](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classHorizontalFilter/#function-getextimatecurrents)**()<br>Returns the currents.  |
| void | **[resetFilter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classHorizontalFilter/#function-resetfilter)**()<br>Reset horizontal filter.  |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| const static int | **[MEAS_LEN](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classHorizontalFilter/#variable-meas-len)**  |
| const static int | **[STATE_LEN](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classHorizontalFilter/#variable-state-len)**  |

## Detailed Description

```cpp
class HorizontalFilter;
```

Horizontal Filter class. 

**Note**: This Class estimates the state of the vehicle in the horizontal frame. The state includes position, velocity, acceleration and currents. The latest state estimate of the filter is obtained using the exposed function getEstimate(). Measurement updates to the filter is done either using measCallback() and the latest state estimate is accessed using getEstimate(). Water currents are estimated but the output is not exposed to the user. 
## Public Functions Documentation

### function HorizontalFilter

```cpp
HorizontalFilter()
```

Contructor Horizontal Filter. 

### function ~HorizontalFilter

```cpp
virtual ~HorizontalFilter() =default
```

Desctructor Horizontal Filter. 

### function computePredict

```cpp
bool computePredict(
    auv_msgs::NavigationStatus & state,
    const ros::Time & t_request
)
```

Propagate the state to the current time. 

**Parameters**: 

  * **state** State vector 
  * **t_request** current time


**Return**: Success or Failure 

### function configure

```cpp
void configure(
    HorizontalFilter::config & configurations
)
```

Configure filter variables and set initialization conditions. 

**Parameters**: 

  * **configurations** 


### function newMeasurement

```cpp
void newMeasurement(
    FilterGimmicks::measurement & m
)
```

Tranforms the measurement from the sensor frame to the filter world frame. Calls addMeasurement and forwardPropagation methods. 

**Parameters**: 

  * **msg** New measurement 


### function deleteMeasurementsInBuffer

```cpp
void deleteMeasurementsInBuffer()
```

Clears all measurements older than a timer period defined in save_meas_interval. 

### function getExtimateCurrents

```cpp
std::vector< double > getExtimateCurrents()
```

Returns the currents. 

**Return**: x_current, y_current 

### function resetFilter

```cpp
void resetFilter()
```

Reset horizontal filter. 

## Public Attributes Documentation

### variable MEAS_LEN

```cpp
static const static int MEAS_LEN = 6;
```


### variable STATE_LEN

```cpp
static const static int STATE_LEN = 8;
```


-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000