---
title: FilterGimmicks
summary: FilterGimmicks namespace. 

---

# FilterGimmicks

[FilterGimmicks]() namespace.  [More...](#detailed-description)

## Classes

|                | Name           |
| -------------- | -------------- |
| struct | **[FilterGimmicks::measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/)** <br>Define a measurement object.  |
| struct | **[FilterGimmicks::predicate_frame_id](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1predicate__frame__id/)** <br>Predicate to find a giver frame_id among a list of frame_id.  |

## Types

|                | Name           |
| -------------- | -------------- |
| typedef struct [FilterGimmicks::measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/) | **[measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Namespaces/namespaceFilterGimmicks/#typedef-measurement)** <br>Define a measurement object.  |

## Functions

|                | Name           |
| -------------- | -------------- |
| bool | **[isinvalid](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Namespaces/namespaceFilterGimmicks/#function-isinvalid)**(const [measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/) & m, double time)<br>Validates a measurement if time is not negative, measurement or covariance are not nan.  |

## Detailed Description

[FilterGimmicks]() namespace. 

**Note**: gimmicks methods for navigation filters 
## Types Documentation

### typedef measurement

```cpp
typedef struct FilterGimmicks::measurement FilterGimmicks::measurement;
```

Define a measurement object. 

**Note**: measurement struct temporary placed here. To be moved later somewhere 


## Functions Documentation

### function isinvalid

```cpp
static bool isinvalid(
    const measurement & m,
    double time
)
```

Validates a measurement if time is not negative, measurement or covariance are not nan. 

**Parameters**: 

  * **val** measurement 
  * **time** 


**Return**: 

  * true if measurement is invalid 
  * false if measurement is valid 






-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000