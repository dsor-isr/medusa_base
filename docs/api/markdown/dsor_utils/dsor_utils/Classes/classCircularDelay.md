---
title: CircularDelay
summary: A class that functions as a sample buffer. 

---

# CircularDelay



A class that functions as a sample buffer.  [More...](#detailed-description)


`#include <circular_buffer.hpp>`

## Public Classes

|                | Name           |
| -------------- | -------------- |
| class | **[const_iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__iterator/)**  |
| class | **[const_reverse_iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/)**  |
| class | **[iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/)**  |
| class | **[reverse_iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/)**  |

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[CircularDelay](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay/#function-circulardelay)**()<br>Constructor that initializes that buffer and its set index.  |
| type | **[push](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay/#function-push)**(type input)<br>With this function you can insert a new sample into the buffer.  |
| type | **[get](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay/#function-get)**(size_t delay)<br>With this function you can retrieve a sample from the past.  |
| [iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/) | **[end](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay/#function-end)**() |
| [iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/) | **[begin](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay/#function-begin)**() |
| [reverse_iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/) | **[rend](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay/#function-rend)**() |
| [reverse_iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/) | **[rbegin](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay/#function-rbegin)**() |

## Detailed Description

```cpp
template <typename type ,
size_t size>
class CircularDelay;
```

A class that functions as a sample buffer. 

**Template Parameters**: 

  * **type** Type of sample that needs to be stored. 
  * **size** Size of how big the history buffer is. 


**Copyright**: GPL V3

Authors: Jimmy van den Berg ([vandenberg.jimmy@gmail.com](mailto:vandenberg.jimmy@gmail.com)) Maintained by: Jimmy van den Berg ([vandenberg.jimmy@gmail.com](mailto:vandenberg.jimmy@gmail.com)) Last Update: 08/02/2018 Github: [https://github.com/jimmyberg/CircularDelay/tree/master](https://github.com/jimmyberg/CircularDelay/tree/master) Brief: Circular delay software library. Here data can be stored and retrieved is a LiFo manner. License: GNU 
Copyright (C) 2018 Jimmy van den Berg

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see [http://www.gnu.org/licenses/](http://www.gnu.org/licenses/).

You can use this to insert samples and use the get function to get a sample from the past.

## Public Functions Documentation

### function CircularDelay

```cpp
CircularDelay()
```

Constructor that initializes that buffer and its set index. 

**Template Parameters**: 

  * **type** Type of sample that needs to be stored. 
  * **size** Size of how big the history buffer is. 


### function push

```cpp
type push(
    type input
)
```

With this function you can insert a new sample into the buffer. 

**Parameters**: 

  * **input** Sample to push into.


**Template Parameters**: 

  * **type** Type of sample that needs to be stored. 
  * **size** Size of how big the history buffer is.


**Return**: Value that has been pushed. 

### function get

```cpp
type get(
    size_t delay
)
```

With this function you can retrieve a sample from the past. 

**Parameters**: 

  * **delay** How many samples you ago you want to get.


**Template Parameters**: 

  * **type** Type of sample that needs to be stored. 
  * **size** Size of how big the history buffer is.


**Return**: The sample of delay ago. 

Maximum delay is the size of the [CircularDelay](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay/) - 1.


### function end

```cpp
inline iterator end()
```


### function begin

```cpp
inline iterator begin()
```


### function rend

```cpp
inline reverse_iterator rend()
```


### function rbegin

```cpp
inline reverse_iterator rbegin()
```


-------------------------------

Updated on 2022-05-30 at 08:04:25 +0000