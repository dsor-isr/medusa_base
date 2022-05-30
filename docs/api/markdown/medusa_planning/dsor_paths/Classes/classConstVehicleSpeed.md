---
title: ConstVehicleSpeed
summary: Class that implements a constant speed value requirement. This class will receive in its constructor the desired velocity for the speed in the inertial frame and will implement the corresponding speed in the path frame. 

---

# ConstVehicleSpeed



Class that implements a constant speed value requirement. This class will receive in its constructor the desired velocity for the speed in the inertial frame and will implement the corresponding speed in the path frame.  [More...](#detailed-description)


`#include <ConstVehicleSpeed.h>`

Inherits from [Speed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[ConstVehicleSpeed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstVehicleSpeed/#function-constvehiclespeed)**(double vehicle_speed, double default_val)<br>Constructor for the [ConstVehicleSpeed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstVehicleSpeed/) class. Receives as a parameter a double which represents the desired speed of the vehicle in the inertial frame.  |
| virtual double | **[getVd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstVehicleSpeed/#function-getvd)**(double gamma, double tangent_norm) override<br>Method to get the desired velocity for the virtual target on the path given the path parameter (given by the value of gamma)  |
| virtual double | **[get_d_Vd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstVehicleSpeed/#function-get-d-vd)**(double gamma, double tangent_norm) override<br>Method to get the desired acceleration for the virtual target on the path given the path parameter (given by the value of gamma)  |
| virtual double | **[getDefaultVd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstVehicleSpeed/#function-getdefaultvd)**(double gamma, double tangent_norm) override<br>Method to get the default desired velocity for safety when we are doing path following and want to have a backup value.  |

## Additional inherited members

**Public Functions inherited from [Speed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/)**

|                | Name           |
| -------------- | -------------- |
| virtual | **[~Speed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/#function-~speed)**()<br>Virtual destructor for the abstract class.  |


## Detailed Description

```cpp
class ConstVehicleSpeed;
```

Class that implements a constant speed value requirement. This class will receive in its constructor the desired velocity for the speed in the inertial frame and will implement the corresponding speed in the path frame. 

**Author**: 

  * Marcelo Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 

This class is used as a part of the speeds library 

## Public Functions Documentation

### function ConstVehicleSpeed

```cpp
ConstVehicleSpeed(
    double vehicle_speed,
    double default_val
)
```

Constructor for the [ConstVehicleSpeed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstVehicleSpeed/) class. Receives as a parameter a double which represents the desired speed of the vehicle in the inertial frame. 

**Parameters**: 

  * **vehicle_speed** The desired speed for the vehicle in m/s 
  * **default_val** The desired default_value when the vehicle is outside the gamma range 


### function getVd

```cpp
virtual double getVd(
    double gamma,
    double tangent_norm
) override
```

Method to get the desired velocity for the virtual target on the path given the path parameter (given by the value of gamma) 

**Parameters**: 

  * **gamma** The path parameter 
  * **tangent_norm** The norm of the tangent to the path


**Return**: A double with the desired speed 

**Reimplements**: [Speed::getVd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/#function-getvd)


### function get_d_Vd

```cpp
virtual double get_d_Vd(
    double gamma,
    double tangent_norm
) override
```

Method to get the desired acceleration for the virtual target on the path given the path parameter (given by the value of gamma) 

**Parameters**: 

  * **gamma** The value of the path parameter 
  * **tangent_norm** The norm of the tangent to the path


**Return**: A double with the desired acceleration 

**Reimplements**: [Speed::get_d_Vd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/#function-get-d-vd)


### function getDefaultVd

```cpp
virtual double getDefaultVd(
    double gamma,
    double tangent_norm
) override
```

Method to get the default desired velocity for safety when we are doing path following and want to have a backup value. 

**Parameters**: 

  * **gamma** The value of the path parameter 
  * **tangent_norm** The norm of the tangent to the path in gamma


**Return**: A double with the default desired speed 

**Reimplements**: [Speed::getDefaultVd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/#function-getdefaultvd)


-------------------------------

Updated on 2022-05-30 at 08:04:27 +0000