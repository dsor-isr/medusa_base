---
title: FilterGimmicks::predicate_frame_id
summary: Predicate to find a giver frame_id among a list of frame_id. 

---

# FilterGimmicks::predicate_frame_id



Predicate to find a giver frame_id among a list of frame_id. 


`#include <FilterGimmicks.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[predicate_frame_id](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1predicate__frame__id/#function-predicate-frame-id)**(std::string target_) |
| bool | **[operator()](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1predicate__frame__id/#function-operator())**([measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/) & sensor) |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| std::string | **[target](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1predicate__frame__id/#variable-target)**  |

## Public Functions Documentation

### function predicate_frame_id

```cpp
inline predicate_frame_id(
    std::string target_
)
```


### function operator()

```cpp
inline bool operator()(
    measurement & sensor
)
```


## Public Attributes Documentation

### variable target

```cpp
std::string target;
```


-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000