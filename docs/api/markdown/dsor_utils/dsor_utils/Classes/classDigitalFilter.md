---
title: DigitalFilter
summary: Abstract base class for digital moving filters. 

---

# DigitalFilter



Abstract base class for digital moving filters.  [More...](#detailed-description)


`#include <digital_filter.hpp>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| virtual Type | **[update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/#function-update)**(Type newValue) =0 |
| virtual Type | **[getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/#function-getoutput)**() =0 |

## Detailed Description

```cpp
template <typename Type >
class DigitalFilter;
```

Abstract base class for digital moving filters. 

**Template Parameters**: 

  * **Type** Floating point type used. 

## Public Functions Documentation

### function update

```cpp
virtual Type update(
    Type newValue
) =0
```


**Reimplemented by**: [HighPassFilter::update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classHighPassFilter/#function-update), [LowPassFilter::update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter/#function-update), [LowPassFilter2::update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter2/#function-update), [HighPassFilter3::update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classHighPassFilter3/#function-update), [LowPassFilter3::update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter3/#function-update), [LowPassFilter3MatchedZ::update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter3MatchedZ/#function-update), [LowPassFilter3DiffApprox::update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter3DiffApprox/#function-update), [Differentiator::update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDifferentiator/#function-update)


### function getOutput

```cpp
virtual Type getOutput() =0
```


**Reimplemented by**: [Differentiator::getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDifferentiator/#function-getoutput), [LowPassFilter::getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter/#function-getoutput), [HighPassFilter::getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classHighPassFilter/#function-getoutput), [LowPassFilter2::getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter2/#function-getoutput), [HighPassFilter3::getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classHighPassFilter3/#function-getoutput), [LowPassFilter3::getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter3/#function-getoutput), [LowPassFilter3MatchedZ::getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter3MatchedZ/#function-getoutput), [LowPassFilter3DiffApprox::getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter3DiffApprox/#function-getoutput)


-------------------------------

Updated on 2022-05-30 at 08:04:25 +0000