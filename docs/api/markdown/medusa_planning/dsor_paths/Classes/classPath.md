---
title: Path
summary: The Path class! This class implements all the meethod to get the desired position, derivative, second derivatives, tangent, curvature and derivative norm. 

---

# Path



The [Path]() class! This class implements all the meethod to get the desired position, derivative, second derivatives, tangent, curvature and derivative norm.  [More...](#detailed-description)


`#include <Path.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[Path](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-path)**()<br>The constructor for the path class.  |
| | **[~Path](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-~path)**()<br>The destructor for the path class. Frees all the memory allocated for each path section inside the vector of section.  |
| bool | **[isEmpty](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-isempty)**()<br>Method to check wether the path contains path sections or not.  |
| bool | **[addPathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-addpathsection)**([PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/) * path_section)<br>Method to add a path section to the path.  |
| bool | **[addSpeedSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-addspeedsection)**([Speed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSpeed/) * speed_section)<br>  * Method to add a speed section to the path  |
| std::tuple< [PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/) *, double > | **[getPathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-getpathsection)**(double gamma)<br>Method to get the path section corresponding to a given gamma.  |
| std::optional< Eigen::Vector3d > | **[eq_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-eq-pd)**(double gamma)<br>  * Method to retrieve the position in the path given the path parameter gamma  |
| std::optional< Eigen::Vector3d > | **[eq_d_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-eq-d-pd)**(double gamma)<br>  * Method to retrieve the derivative of the position in the path given the path parameter gamma  |
| std::optional< Eigen::Vector3d > | **[eq_dd_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-eq-dd-pd)**(double gamma)<br>  * Method to retrieve the second derivative of the position in the path given the path parameter gamma  |
| std::optional< double > | **[tangent](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-tangent)**(double gamma)<br>  * Get the tangent to the path, given the gamma parameter  |
| std::optional< double > | **[curvature](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-curvature)**(double gamma)<br>  * The curvature of the path, given the gamma parameter  |
| std::optional< double > | **[derivative_norm](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-derivative-norm)**(double gamma)<br>  * The norm of the derivative of the path  |
| std::optional< double > | **[eq_vd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-eq-vd)**(double gamma)<br>Method to get the desired speed profile for a particular part of the path given the path parameter gamma.  |
| std::optional< double > | **[eq_d_vd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-eq-d-vd)**(double gamma)<br>Method to get the desired acceleration profile for a paritcular part of the path given the path parameter.  |
| std::pair< double, double > | **[getMinMaxGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-getminmaxgamma)**()<br>Method to retrieve the minimum and maximum gamma values allowed for the current path.  |
| std::optional< double > | **[getClosestGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#function-getclosestgamma)**(Eigen::Vector3d & coordinate)<br>Method to get the gamma corresponding to the closest point on the path, relative to the coordinate passed as argument.  |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| std::vector< [PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/) * > | **[sections_](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/#variable-sections-)** <br>List of PathSections to follow.  |

## Detailed Description

```cpp
class Path;
```

The [Path]() class! This class implements all the meethod to get the desired position, derivative, second derivatives, tangent, curvature and derivative norm. 

**Author**: 

  * Marcelo Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 

This class stores a list of sections, and switches between sections based on the gamma value passed to the functions

This class has the option to have a single section (for example, appropriate for a [Bernoulli](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classBernoulli/)) and the option to have multiple section (for example, lines and arcs to make lawnmowers) 

## Public Functions Documentation

### function Path

```cpp
Path()
```

The constructor for the path class. 

### function ~Path

```cpp
~Path()
```

The destructor for the path class. Frees all the memory allocated for each path section inside the vector of section. 

Destructor for the [Path](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/) object. Deletes the memory allocated for each path section inside the vector of sections. 


### function isEmpty

```cpp
bool isEmpty()
```

Method to check wether the path contains path sections or not. 

**Return**: A boolean true if the path contains path sections in the vector 

### function addPathSection

```cpp
bool addPathSection(
    PathSection * path_section
)
```

Method to add a path section to the path. 

**Parameters**: 

  * ***path_section** A Pointer to a [PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/) object (can be [Line](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classLine/), [Arc2D](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classArc2D/), [Bernoulli](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classBernoulli/), etc...)


**Return**: true if path was added with success or false if not 

### function addSpeedSection

```cpp
bool addSpeedSection(
    Speed * speed_section
)
```

  * Method to add a speed section to the path 

**Parameters**: 

  * **speed_section** A Pointer to a SpeedSection object (can be [ConstRabbitSpeed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstRabbitSpeed/), [ConstVehicleSpeed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classConstVehicleSpeed/), ...)


**Return**: true if the speed was added with success or false if not 

### function getPathSection

```cpp
std::tuple< PathSection *, double > getPathSection(
    double gamma
)
```

Method to get the path section corresponding to a given gamma. 

**Parameters**: 

  * **gamma** - A double with the path parameter


**Return**: The [PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/) object corresponding to the given gamma and the internal gamma corresponding to that path section 

### function eq_pd

```cpp
std::optional< Eigen::Vector3d > eq_pd(
    double gamma
)
```

  * Method to retrieve the position in the path given the path parameter gamma 

**Parameters**: 

  * **gamma** - A double with the path parameter


**Return**: A vector (3x1) with [x(gamma), y(gamma), z(gamma)] 

### function eq_d_pd

```cpp
std::optional< Eigen::Vector3d > eq_d_pd(
    double gamma
)
```

  * Method to retrieve the derivative of the position in the path given the path parameter gamma 

**Parameters**: 

  * **gamma** - A double with the path parameter


**Return**: A vector (3x1) with [x_dot(gamma, y_dot(gamma), z_dot(gamma)] 

### function eq_dd_pd

```cpp
std::optional< Eigen::Vector3d > eq_dd_pd(
    double gamma
)
```

  * Method to retrieve the second derivative of the position in the path given the path parameter gamma 

**Parameters**: 

  * **gamma** - A double with the path parameter


**Return**: A vector (3x1) with [x_ddot(gamma), y_ddot(gamma), z_ddot(gamma)] 

### function tangent

```cpp
std::optional< double > tangent(
    double gamma
)
```

  * Get the tangent to the path, given the gamma parameter 

**Parameters**: 

  * **gamma** - A double with the path parameter


**Return**: A double with the tangent size 

### function curvature

```cpp
std::optional< double > curvature(
    double gamma
)
```

  * The curvature of the path, given the gamma parameter 

**Parameters**: 

  * **gamma** - A double with the path parameter


**Return**: A double with the curvature of the path 

### function derivative_norm

```cpp
std::optional< double > derivative_norm(
    double gamma
)
```

  * The norm of the derivative of the path 

**Parameters**: 

  * **gamma** - A double with the path parameter


**Return**: A double with the norm of the derivative of the path 

### function eq_vd

```cpp
std::optional< double > eq_vd(
    double gamma
)
```

Method to get the desired speed profile for a particular part of the path given the path parameter gamma. 

**Parameters**: 

  * **gamma** A double with the path parameter


**Return**: A double with the value of vd 

### function eq_d_vd

```cpp
std::optional< double > eq_d_vd(
    double gamma
)
```

Method to get the desired acceleration profile for a paritcular part of the path given the path parameter. 

**Parameters**: 

  * **gamma** A double with the path parameter


**Return**: A double with the valud of d_vd 

### function getMinMaxGamma

```cpp
std::pair< double, double > getMinMaxGamma()
```

Method to retrieve the minimum and maximum gamma values allowed for the current path. 

**Return**: A pair with 2 doubles with the first being the minimum value and the second the maximum value 

Method to return the minimum and maximum value that gamma can achive in the path.


### function getClosestGamma

```cpp
std::optional< double > getClosestGamma(
    Eigen::Vector3d & coordinate
)
```

Method to get the gamma corresponding to the closest point on the path, relative to the coordinate passed as argument. 

**Parameters**: 

  * **coordinate** The coordinate of the vehicle


**Return**: A double with the gamma of the path corresponding to the closest point on the path 

## Public Attributes Documentation

### variable sections_

```cpp
std::vector< PathSection * > sections_;
```

List of PathSections to follow. 

-------------------------------

Updated on 2022-05-30 at 08:04:27 +0000