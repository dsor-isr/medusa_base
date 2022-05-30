---
title: HighPassFilter
summary: Class for high pass filter using bilinear transform. 

---

# HighPassFilter



Class for high pass filter using bilinear transform.  [More...](#detailed-description)


`#include <highpass_filter.hpp>`

Inherits from [DigitalFilter< float >](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[HighPassFilter](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classHighPassFilter/#function-highpassfilter)**(float idt, float omega_c)<br>Constructor to set sample time and the tau constant.  |
| virtual float | **[update](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classHighPassFilter/#function-update)**(float newValue)<br>Update function to push new value into the low pass filter.  |
| virtual float | **[getOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classHighPassFilter/#function-getoutput)**()<br>Gets the output.  |
| void | **[configOutput](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classHighPassFilter/#function-configoutput)**(float newOutput)<br>Force the output to a desired value.  |
| const float * | **[outputPointer](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classHighPassFilter/#function-outputpointer)**() |

## Detailed Description

```cpp
class HighPassFilter;
```

Class for high pass filter using bilinear transform. 

Authors: Jimmy van den Berg ([vandenberg.jimmy@gmail.com](mailto:vandenberg.jimmy@gmail.com)) Maintained by: Jimmy van den Berg ([vandenberg.jimmy@gmail.com](mailto:vandenberg.jimmy@gmail.com)) Last Update: 08/02/2018 Github: [https://github.com/jimmyberg/DigitalFilters](https://github.com/jimmyberg/DigitalFilters) License: GNU Brief: Multiple classes for low pass filters of different orders, inherited from the [DigitalFilter](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classDigitalFilter/) class of moving filters. A low-pass filter is a filter that passes signals with a frequency lower than a selected cutoff frequency and attenuates signals with frequencies higher than the cutoff frequency. 

## Public Functions Documentation

### function HighPassFilter

```cpp
inline HighPassFilter(
    float idt,
    float omega_c
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