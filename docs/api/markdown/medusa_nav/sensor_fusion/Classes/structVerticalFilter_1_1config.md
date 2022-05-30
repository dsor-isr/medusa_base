---
title: VerticalFilter::config

---

# VerticalFilter::config





## Public Attributes

|                | Name           |
| -------------- | -------------- |
| bool | **[initialized](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/#variable-initialized)**  |
| bool | **[broadcast_tf](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/#variable-broadcast-tf)**  |
| double | **[vertical_drag](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/#variable-vertical-drag)** <br>alpha, beta, bouyancy  |
| double | **[reject_counter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/#variable-reject-counter)**  |
| double | **[init_cov](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/#variable-init-cov)**  |
| double | **[process_noise](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/#variable-process-noise)**  |
| double | **[kalman_config](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/#variable-kalman-config)**  |
| std::vector< std::string > | **[frames](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/#variable-frames)** <br>base_frame, odom_frame, map_frame, world_frame  |
| [FilterGimmicks::measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/) | **[meas_init](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/#variable-meas-init)**  |
| std::vector< [FilterGimmicks::measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/) > | **[sensors](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/#variable-sensors)**  |
| tf2_ros::TransformBroadcaster * | **[br_node](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/#variable-br-node)**  |

## Public Attributes Documentation

### variable initialized

```cpp
bool initialized {false};
```


### variable broadcast_tf

```cpp
bool broadcast_tf;
```


### variable vertical_drag

```cpp
double vertical_drag;
```

alpha, beta, bouyancy 

### variable reject_counter

```cpp
double reject_counter;
```


### variable init_cov

```cpp
double init_cov;
```


### variable process_noise

```cpp
double process_noise;
```


### variable kalman_config

```cpp
double kalman_config;
```


### variable frames

```cpp
std::vector< std::string > frames;
```

base_frame, odom_frame, map_frame, world_frame 

### variable meas_init

```cpp
FilterGimmicks::measurement meas_init;
```


### variable sensors

```cpp
std::vector< FilterGimmicks::measurement > sensors;
```


### variable br_node

```cpp
tf2_ros::TransformBroadcaster * br_node;
```


-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000