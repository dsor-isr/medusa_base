---
title: LowPassFilter3MatchedZ
summary: Class for third order low pass filter. This is designed using the matched Z transform. 

---

# LowPassFilter3MatchedZ



Class for third order low pass filter. This is designed using the matched Z transform. 


`#include <lowpass_filter.hpp>`

Inherits from [DigitalFilter< float >](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[LowPassFilter3MatchedZ](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter3MatchedZ/#function-lowpassfilter3matchedz)**(float sampleTime, float omega_c)<br>Constructor to set sample time and the tau constant.  |
| virtual float | **[update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter3MatchedZ/#function-update)**(float newValue)<br>Update function to push new value into the filter.  |
| virtual float | **[getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter3MatchedZ/#function-getoutput)**()<br>Gets the output.  |

## Public Functions Documentation

### function LowPassFilter3MatchedZ

```cpp
inline LowPassFilter3MatchedZ(
    float sampleTime,
    float omega_c
)
```

Constructor to set sample time and the tau constant. 

**Parameters**: 

  * **sampleTime** Sample time for the low pass filter 
  * **omega_c** Cutoff angular frequency \( \omega_c = 2 pi f_c\) where \( f_c \) is the cutoff frequency 


### function update

```cpp
inline virtual float update(
    float newValue
)
```

Update function to push new value into the filter. 

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