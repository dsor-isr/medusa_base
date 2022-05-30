---
title: Section
summary: Contains Section variables for path followin algorithms. 

---

# Section



Contains [Section]() variables for path followin algorithms.  [More...](#detailed-description)


`#include <Section.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[Section](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#function-section)**() |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| int | **[type](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-type)** <br>1= WP; 2=Line; 3=Arc; 4=Depth  |
| double | **[xi](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-xi)** <br>initial x of section  |
| double | **[yi](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-yi)** <br>initial y of section  |
| double | **[xc](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-xc)** <br>x of center of arc (-1 if line or point)  |
| double | **[yc](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-yc)** <br>y of center of arc (-1 if line or point)  |
| double | **[xe](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-xe)** <br>ending x of section  |
| double | **[ye](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-ye)** <br>ending y of section  |
| float | **[velocity](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-velocity)** <br>velocity desired of the vehicle  |
| int | **[adirection](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-adirection)** <br>-1 if vehicle is turning clockwise, -1 otherwise (only applied to arcs)  |
| float | **[radius](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-radius)** <br>adius of the arc  |
| float | **[heading](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-heading)** <br>yaw of the vehicle  |
| float | **[time](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-time)** <br>only used for point, depth and alt, time to use the reference  |
| int | **[nVehicle](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-nvehicle)** <br>number of the vehicle (possible id)  |
| double | **[gamma_s](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-gamma-s)** <br>Starting gamma (not normalized)  |
| double | **[gamma_e](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-gamma-e)** <br>Ending gamma (not normalized)  |
| float | **[depth](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/#variable-depth)** <br>Depth of the section.  |

## Detailed Description

```cpp
class Section;
```

Contains [Section]() variables for path followin algorithms. 

**Note**: nVehicle is always -1, probably not being used 
## Public Functions Documentation

### function Section

```cpp
inline Section()
```


## Public Attributes Documentation

### variable type

```cpp
int type;
```

1= WP; 2=Line; 3=Arc; 4=Depth 

### variable xi

```cpp
double xi;
```

initial x of section 

### variable yi

```cpp
double yi;
```

initial y of section 

### variable xc

```cpp
double xc;
```

x of center of arc (-1 if line or point) 

### variable yc

```cpp
double yc;
```

y of center of arc (-1 if line or point) 

### variable xe

```cpp
double xe;
```

ending x of section 

### variable ye

```cpp
double ye;
```

ending y of section 

### variable velocity

```cpp
float velocity;
```

velocity desired of the vehicle 

### variable adirection

```cpp
int adirection;
```

-1 if vehicle is turning clockwise, -1 otherwise (only applied to arcs) 

### variable radius

```cpp
float radius;
```

adius of the arc 

### variable heading

```cpp
float heading;
```

yaw of the vehicle 

### variable time

```cpp
float time;
```

only used for point, depth and alt, time to use the reference 

### variable nVehicle

```cpp
int nVehicle;
```

number of the vehicle (possible id) 

### variable gamma_s

```cpp
double gamma_s;
```

Starting gamma (not normalized) 

### variable gamma_e

```cpp
double gamma_e;
```

Ending gamma (not normalized) 

### variable depth

```cpp
float depth;
```

Depth of the section. 

-------------------------------

Updated on 2022-05-30 at 08:04:30 +0000