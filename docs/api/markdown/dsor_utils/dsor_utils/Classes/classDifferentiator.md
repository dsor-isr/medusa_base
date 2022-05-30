---
title: Differentiator
summary: Class for the differentiator. 

---

# Differentiator



Class for the differentiator.  [More...](#detailed-description)


`#include <differentiator.hpp>`

Inherits from [DigitalFilter< T >](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[Differentiator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDifferentiator/#function-differentiator)**(T sampleTime)<br>Constructor to set sample time and the tau constant.  |
| virtual T | **[update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDifferentiator/#function-update)**(T input)<br>Update function to push new value into the differentiator.  |
| virtual T | **[getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDifferentiator/#function-getoutput)**()<br>Gets the output.  |

## Detailed Description

```cpp
template <typename T >
class Differentiator;
```

Class for the differentiator. 

Authors: Jimmy van den Berg ([vandenberg.jimmy@gmail.com](mailto:vandenberg.jimmy@gmail.com)) Maintained by: Jimmy van den Berg ([vandenberg.jimmy@gmail.com](mailto:vandenberg.jimmy@gmail.com)) Last Update: 08/02/2018 Github: [https://github.com/jimmyberg/DigitalFilters](https://github.com/jimmyberg/DigitalFilters) License: GNU Brief: Class for a differentiator, inherited from the [DigitalFilter](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/) class of moving filters. A differentiator is a filter that is designed such that the output is approximately directly proportionalto the rate of change (the time derivative) of the input. 

## Public Functions Documentation

### function Differentiator

```cpp
inline Differentiator(
    T sampleTime
)
```

Constructor to set sample time and the tau constant. 

**Parameters**: 

  * **sampleTime** Sample time for the low pass filter 


### function update

```cpp
inline virtual T update(
    T input
)
```

Update function to push new value into the differentiator. 

**Parameters**: 

  * **input** The new value after dt time


**Return**: The new output value 

**Reimplements**: [DigitalFilter::update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/#function-update)


### function getOutput

```cpp
inline virtual T getOutput()
```

Gets the output. 

**Return**: The output. 

**Reimplements**: [DigitalFilter::getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/#function-getoutput)


-------------------------------

Updated on 2022-05-30 at 08:04:25 +0000