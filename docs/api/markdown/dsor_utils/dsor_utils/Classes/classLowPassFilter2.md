---
title: LowPassFilter2
summary: Class for a 2nd order low pass filter. 

---

# LowPassFilter2



Class for a 2nd order low pass filter.  [More...](#detailed-description)


`#include <lowpass_filter.hpp>`

Inherits from [DigitalFilter< float >](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[LowPassFilter2](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter2/#function-lowpassfilter2)**(float dt, float tau_c, float ioutput =0)<br>Constructor to set sample time and the tau constant.  |
| virtual float | **[update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter2/#function-update)**(float newValue)<br>Update function to push new value into the filter.  |
| virtual float | **[getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter2/#function-getoutput)**()<br>Gets the output.  |
| void | **[configOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter2/#function-configoutput)**(float newOutput)<br>Force the output to a desired value.  |

## Detailed Description

```cpp
class LowPassFilter2;
```

Class for a 2nd order low pass filter. 



```
        Design to be a 2nd order Butterworth low pass filter.
        Transformation done using the bilinear transform method
```

## Public Functions Documentation

### function LowPassFilter2

```cpp
inline LowPassFilter2(
    float dt,
    float tau_c,
    float ioutput =0
)
```

Constructor to set sample time and the tau constant. 

**Parameters**: 

  * **dt** Sample time for the low pass filter 
  * **tau_c** The time constant for the filter \( \tau_c = \frac{1}{2 pi f_c}\), where \( f_c \) is the cutoff frequency 


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


### function configOutput

```cpp
inline void configOutput(
    float newOutput
)
```

Force the output to a desired value. 

**Parameters**: 

  * **newOutput** The new output 




```
        This can be useful when the output needs to be forced in case
        of extreme inputs or such
```


-------------------------------

Updated on 2022-05-30 at 08:04:25 +0000