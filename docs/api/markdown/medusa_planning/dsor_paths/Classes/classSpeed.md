---
title: Speed
summary: Abstract class to serve as the base for speed as a function of gamma. 

---

# Speed



Abstract class to serve as the base for speed as a function of gamma.  [More...](#detailed-description)


`#include <Speed.h>`

Inherited by [ConstRabbitSpeed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstRabbitSpeed/), [ConstVehicleSpeed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstVehicleSpeed/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| virtual double | **[getVd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/#function-getvd)**(double gamma, double tangent_norm) =0<br>Method to get the desired velocity for the virtual target on the path given the path parameter (given by the value of gamma)  |
| virtual double | **[get_d_Vd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/#function-get-d-vd)**(double gamma, double tangent_norm) =0<br>Method to get the desired acceleration for the virtual target on the path given the path parameter (given by the value of gamma)  |
| virtual double | **[getDefaultVd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/#function-getdefaultvd)**(double gamma, double tangent_norm) =0<br>Method to get the default desired velocity for safety when we are doing path following and want to have a backup value.  |
| virtual | **[~Speed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/#function-~speed)**()<br>Virtual destructor for the abstract class.  |

## Detailed Description

```cpp
class Speed;
```

Abstract class to serve as the base for speed as a function of gamma. 

**Author**: 

  * Marcelo Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 

Since this class is abstract it cannot be instantiated. It must be inherited. 

## Public Functions Documentation

### function getVd

```cpp
virtual double getVd(
    double gamma,
    double tangent_norm
) =0
```

Method to get the desired velocity for the virtual target on the path given the path parameter (given by the value of gamma) 

**Parameters**: 

  * **gamma** The path parameter 
  * **tangent_norm** The norm of the tangent to the path in gamma


**Return**: A double with the desired speed

**Reimplemented by**: [ConstRabbitSpeed::getVd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstRabbitSpeed/#function-getvd), [ConstVehicleSpeed::getVd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstVehicleSpeed/#function-getvd)



NOTE: This method is pure virtual which means it must be implemented by a class that inherits [Speed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/)


### function get_d_Vd

```cpp
virtual double get_d_Vd(
    double gamma,
    double tangent_norm
) =0
```

Method to get the desired acceleration for the virtual target on the path given the path parameter (given by the value of gamma) 

**Parameters**: 

  * **gamma** The value of the path parameter 
  * **tangent_norm** The norm of the tangent to the path in gamma


**Return**: A double with the desired acceleration

**Reimplemented by**: [ConstRabbitSpeed::get_d_Vd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstRabbitSpeed/#function-get-d-vd), [ConstVehicleSpeed::get_d_Vd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstVehicleSpeed/#function-get-d-vd)



NOTE: This method is pure virtual which means it must be implemented by a class that inherits [Speed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/)


### function getDefaultVd

```cpp
virtual double getDefaultVd(
    double gamma,
    double tangent_norm
) =0
```

Method to get the default desired velocity for safety when we are doing path following and want to have a backup value. 

**Parameters**: 

  * **gamma** The value of the path parameter 
  * **tangent_norm** The norm of the tangent to the path in gamma


**Return**: A double with the default desired speed

**Reimplemented by**: [ConstRabbitSpeed::getDefaultVd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstRabbitSpeed/#function-getdefaultvd), [ConstVehicleSpeed::getDefaultVd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstVehicleSpeed/#function-getdefaultvd)



NOTE: THis method is pura virtual which means it must be implemented by a class that inherits [Speed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/)


### function ~Speed

```cpp
virtual ~Speed()
```

Virtual destructor for the abstract class. 

-------------------------------

Updated on 2022-05-30 at 08:04:27 +0000