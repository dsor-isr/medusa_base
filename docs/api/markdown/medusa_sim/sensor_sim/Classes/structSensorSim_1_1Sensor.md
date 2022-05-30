---
title: SensorSim::Sensor

---

# SensorSim::Sensor





## Public Types

|                | Name           |
| -------------- | -------------- |
| enum| **[Type](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#enum-type)** { null, AHRS, DVL_BT, DVL_WT, DEPTH, ALTIMETER, GNSS, RANGE, MODEL} |

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[Sensor](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#function-sensor)**() |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| Type | **[type](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-type)**  |
| std::string | **[frame_id](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-frame-id)**  |
| bool | **[debug](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-debug)**  |
| int | **[count](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-count)**  |
| int | **[thresh](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-thresh)**  |
| double | **[last_update](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-last-update)**  |
| double | **[frequency](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-frequency)**  |
| double | **[variance](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-variance)**  |
| double | **[noise](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-noise)**  |
| double | **[beacon](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-beacon)**  |
| int | **[zone](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-zone)**  |
| bool | **[northp](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-northp)**  |
| double | **[altitude](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-altitude)**  |
| std::map< std::string, Type > | **[enum_map](/medusa_base/api/markdown/medusa_sim/sensor_sim/Classes/structSensorSim_1_1Sensor/#variable-enum-map)**  |

## Public Types Documentation

### enum Type

| Enumerator | Value | Description |
| ---------- | ----- | ----------- |
| null | |   |
| AHRS | |   |
| DVL_BT | |   |
| DVL_WT | |   |
| DEPTH | |   |
| ALTIMETER | |   |
| GNSS | |   |
| RANGE | |   |
| MODEL | |   |




## Public Functions Documentation

### function Sensor

```cpp
inline Sensor()
```


## Public Attributes Documentation

### variable type

```cpp
Type type;
```


### variable frame_id

```cpp
std::string frame_id;
```


### variable debug

```cpp
bool debug;
```


### variable count

```cpp
int count;
```


### variable thresh

```cpp
int thresh;
```


### variable last_update

```cpp
double last_update;
```


### variable frequency

```cpp
double frequency;
```


### variable variance

```cpp
double variance;
```


### variable noise

```cpp
double noise;
```


### variable beacon

```cpp
double beacon;
```


### variable zone

```cpp
int zone;
```


### variable northp

```cpp
bool northp;
```


### variable altitude

```cpp
double altitude;
```


### variable enum_map

```cpp
std::map< std::string, Type > enum_map;
```


-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000