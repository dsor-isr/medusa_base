---
title: MovingAvarageFilter

---

# MovingAvarageFilter



 [More...](#detailed-description)


`#include <moving_average_filter.hpp>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| double | **[update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classMovingAvarageFilter/#function-update)**(double input)<br>Update function to push new value into the moving average filter.  |

## Detailed Description

```cpp
template <size_t size>
class MovingAvarageFilter;
```


Authors: Jimmy van den Berg ([vandenberg.jimmy@gmail.com](mailto:vandenberg.jimmy@gmail.com)) Maintained by: Jimmy van den Berg ([vandenberg.jimmy@gmail.com](mailto:vandenberg.jimmy@gmail.com)) Last Update: 08/02/2018 Github: [https://github.com/jimmyberg/DigitalFilters](https://github.com/jimmyberg/DigitalFilters) License: GNU Brief: Class for a moving average filter, inherited from the [DigitalFilter](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/) class of moving filters. A moving average filter is operates by averaging a number of points from the input signal to produce each point in the output signal. 

## Public Functions Documentation

### function update

```cpp
inline double update(
    double input
)
```

Update function to push new value into the moving average filter. 

**Parameters**: 

  * **input** The new value after dt time


**Return**: The new output value 

-------------------------------

Updated on 2022-05-30 at 08:04:25 +0000