---
title: LowPassFilter3DiffApprox
summary: Class for third order high pass filter. This is designed using the approximated differtial approuch where s=(Z-1)/(Z*T). 

---

# LowPassFilter3DiffApprox



Class for third order high pass filter. This is designed using the approximated differtial approuch where s=(Z-1)/(Z*T). 


`#include <lowpass_filter.hpp>`

Inherits from [DigitalFilter< float >](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[LowPassFilter3DiffApprox](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter3DiffApprox/#function-lowpassfilter3diffapprox)**(float sampleTime, float omega_c, float ioutput =0)<br>Constructor to set sample time and the tau constant.  |
| virtual float | **[update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter3DiffApprox/#function-update)**(float newValue)<br>Update function to push new value into the filter.  |
| virtual float | **[getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter3DiffApprox/#function-getoutput)**()<br>Gets the output.  |

## Public Functions Documentation

### function LowPassFilter3DiffApprox

```cpp
inline LowPassFilter3DiffApprox(
    float sampleTime,
    float omega_c,
    float ioutput =0
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