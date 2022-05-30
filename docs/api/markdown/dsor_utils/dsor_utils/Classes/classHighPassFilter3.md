---
title: HighPassFilter3
summary: Class for third order high pass filter. This is designed using the bilinear transform. 

---

# HighPassFilter3



Class for third order high pass filter. This is designed using the bilinear transform. 


`#include <highpass_filter.hpp>`

Inherits from [DigitalFilter< float >](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[HighPassFilter3](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classHighPassFilter3/#function-highpassfilter3)**(float sampleTime, float omega_c, float ioutput =0)<br>Constructor to set sample time and the tau constant.  |
| virtual float | **[update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classHighPassFilter3/#function-update)**(float newValue)<br>Update function to push new value into the low pass filter.  |
| virtual float | **[getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classHighPassFilter3/#function-getoutput)**()<br>Gets the output.  |

## Public Functions Documentation

### function HighPassFilter3

```cpp
inline HighPassFilter3(
    float sampleTime,
    float omega_c,
    float ioutput =0
)
```

Constructor to set sample time and the tau constant. 

**Parameters**: 

  * **idt** Sample time for the low pass filter 
  * **omega_c** Cutoff angular frequency \( \omega_c = 2 pi f_c\) where \( f_c \) is the cutoff frequency 


### function update

```cpp
inline virtual float update(
    float newValue
)
```

Update function to push new value into the low pass filter. 

**Parameters**: 

  * **newValue** The new value after dt time


**Return**: The new output value 

**Reimplements**: [DigitalFilter::update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/#function-update)


### function getOutput

```cpp
inline virtual float getOutput()
```

Gets the output. 

**Return**: The output. 

**Reimplements**: [DigitalFilter::getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/#function-getoutput)


-------------------------------

Updated on 2022-05-30 at 08:04:25 +0000