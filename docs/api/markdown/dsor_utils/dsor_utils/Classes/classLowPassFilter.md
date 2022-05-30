---
title: LowPassFilter
summary: Class for a low pass filter. 

---

# LowPassFilter



Class for a low pass filter.  [More...](#detailed-description)


`#include <lowpass_filter.hpp>`

Inherits from [DigitalFilter< float >](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[LowPassFilter](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter/#function-lowpassfilter)**(float idt, float omega_c, float ioutput =0)<br>Constructor to set sample time and the tau constant.  |
| virtual float | **[update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter/#function-update)**(float newValue)<br>Update function to push new value into the low pass filter.  |
| virtual float | **[getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter/#function-getoutput)**()<br>Gets the output.  |
| void | **[configOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter/#function-configoutput)**(float newOutput)<br>Force the output to a desired value.  |
| const float * | **[outputPointer](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classLowPassFilter/#function-outputpointer)**() |

## Detailed Description

```cpp
class LowPassFilter;
```

Class for a low pass filter. 

Authors: Jimmy van den Berg ([vandenberg.jimmy@gmail.com](mailto:vandenberg.jimmy@gmail.com)) Maintained by: Jimmy van den Berg ([vandenberg.jimmy@gmail.com](mailto:vandenberg.jimmy@gmail.com)) Last Update: 08/02/2018 Github: [https://github.com/jimmyberg/DigitalFilters](https://github.com/jimmyberg/DigitalFilters) License: GNU Brief: Multiple classes for low pass filters of different orders, inherited from the [DigitalFilter](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/) class of moving filters. A low-pass filter is a filter that passes signals with a frequency lower than a selected cutoff frequency and attenuates signals with frequencies higher than the cutoff frequency. 

```
        Design to be a first order Butterworth low pass filter.
        Transformation done using the matched-Z-transform method
```

## Public Functions Documentation

### function LowPassFilter

```cpp
inline LowPassFilter(
    float idt,
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


### function outputPointer

```cpp
inline const float * outputPointer()
```


-------------------------------

Updated on 2022-05-30 at 08:04:25 +0000