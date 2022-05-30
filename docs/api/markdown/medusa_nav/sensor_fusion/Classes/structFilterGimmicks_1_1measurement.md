---
title: FilterGimmicks::measurement
summary: Define a measurement object. 

---

# FilterGimmicks::measurement



Define a measurement object.  [More...](#detailed-description)


`#include <FilterGimmicks.h>`

## Public Types

|                | Name           |
| -------------- | -------------- |
| enum| **[measurement_type](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#enum-measurement-type)** { Null = 1, CONFIG, VALUE} |

## Public Functions

|                | Name           |
| -------------- | -------------- |
| void | **[setLength](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#function-setlength)**(int length) |
| | **[measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#function-measurement)**() |
| | **[measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#function-measurement)**(int length) |
| | **[measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#function-measurement)**(std::string header_) |
| | **[measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#function-measurement)**(std::string frame_id, std::string config_, std::vector< double > noise_, double rejection_threshold_, int reject_counter_, double outlier_increase_) |
| | **[measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#function-measurement)**(std::string frame_id, std::vector< double > value_, std::vector< double > noise_) |
| | **[measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#function-measurement)**(const dsor_msgs::Measurement & m_) |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| bool | **[base_frame](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#variable-base-frame)**  |
| std_msgs::Header | **[header](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#variable-header)**  |
| Eigen::VectorXd | **[value](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#variable-value)**  |
| Eigen::VectorXd | **[noise](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#variable-noise)**  |
| Eigen::VectorXd | **[config](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#variable-config)**  |
| Eigen::VectorXd | **[state_copy](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#variable-state-copy)**  |
| Eigen::MatrixXd | **[state_cov_copy](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#variable-state-cov-copy)**  |
| std::string | **[sensor_config](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#variable-sensor-config)**  |
| double | **[outlier_tolerance](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#variable-outlier-tolerance)**  |
| double | **[time_of_previous_meas](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#variable-time-of-previous-meas)**  |
| double | **[outlier_increase](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#variable-outlier-increase)**  |
| int | **[reject_counter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/#variable-reject-counter)**  |

## Detailed Description

```cpp
struct FilterGimmicks::measurement;
```

Define a measurement object. 

**Note**: measurement struct temporary placed here. To be moved later somewhere 
## Public Types Documentation

### enum measurement_type

| Enumerator | Value | Description |
| ---------- | ----- | ----------- |
| Null | 1|   |
| CONFIG | |   |
| VALUE | |   |




## Public Functions Documentation

### function setLength

```cpp
inline void setLength(
    int length
)
```


### function measurement

```cpp
inline measurement()
```


### function measurement

```cpp
inline measurement(
    int length
)
```


### function measurement

```cpp
inline measurement(
    std::string header_
)
```


### function measurement

```cpp
inline measurement(
    std::string frame_id,
    std::string config_,
    std::vector< double > noise_,
    double rejection_threshold_,
    int reject_counter_,
    double outlier_increase_
)
```


### function measurement

```cpp
inline measurement(
    std::string frame_id,
    std::vector< double > value_,
    std::vector< double > noise_
)
```


### function measurement

```cpp
inline measurement(
    const dsor_msgs::Measurement & m_
)
```


## Public Attributes Documentation

### variable base_frame

```cpp
bool base_frame;
```


### variable header

```cpp
std_msgs::Header header;
```


### variable value

```cpp
Eigen::VectorXd value;
```


### variable noise

```cpp
Eigen::VectorXd noise;
```


### variable config

```cpp
Eigen::VectorXd config;
```


### variable state_copy

```cpp
Eigen::VectorXd state_copy;
```


### variable state_cov_copy

```cpp
Eigen::MatrixXd state_cov_copy;
```


### variable sensor_config

```cpp
std::string sensor_config;
```


### variable outlier_tolerance

```cpp
double outlier_tolerance;
```


### variable time_of_previous_meas

```cpp
double time_of_previous_meas;
```


### variable outlier_increase

```cpp
double outlier_increase;
```


### variable reject_counter

```cpp
int reject_counter;
```


-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000