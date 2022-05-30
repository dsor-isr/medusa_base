---
title: Circle2D
summary: Class that implements a 2D Circle section. 

---

# Circle2D



Class that implements a 2D Circle section.  [More...](#detailed-description)


`#include <Circle2D.h>`

Inherits from [PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[Circle2D](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/#function-circle2d)**(double radius, double center_x, double center_y)<br>Constructor for the path section that assumes the plane in which the path is placed is z=0.  |
| | **[Circle2D](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/#function-circle2d)**(double radius, double center_x, double center_y, double z)<br>Constructor for the path section.  |
| virtual Eigen::Vector3d | **[eq_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/#function-eq-pd)**(double t) override<br>[Path](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/) Section equation.  |
| virtual Eigen::Vector3d | **[eq_d_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/#function-eq-d-pd)**(double t) override<br>First derivative of the path section equation with respect to the path parameter.  |
| virtual Eigen::Vector3d | **[eq_dd_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/#function-eq-dd-pd)**(double t) override<br>Second derivative of the path section equation with respect to the path parameter t.  |
| virtual double | **[curvature](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/#function-curvature)**(double t) override<br>Compute the curvature using only the radius. Computed as 1/Radius.  |
| virtual double | **[getClosestPointGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/#function-getclosestpointgamma)**(Eigen::Vector3d & coordinate) override<br>Method for getting the gamma of the closest point.  |

## Additional inherited members

**Public Functions inherited from [PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/)**

|                | Name           |
| -------------- | -------------- |
| virtual double | **[tangent](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-tangent)**(double t)<br>Default method for computing the tangent to the path section.  |
| virtual double | **[derivative_norm](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-derivative-norm)**(double t)<br>Default method for computing the norm of the derivative.  |
| bool | **[can_be_composed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-can-be-composed)**()<br>Method to return whether a pathSection can be composed with other path sections or not.  |
| double | **[limitGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-limitgamma)**(double t)<br>Method to limit the gamma between the minimum and maximum value    By default the maximum gamma is a very high number.  |
| double | **[getMaxGammaValue](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-getmaxgammavalue)**()<br>Method used to get the maximum gamma allowed by the path section By the default is the maximum valid number possible in c++.  |
| double | **[getMinGammaValue](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-getmingammavalue)**()<br>Method used to get the minimum gamma allowed by the path section By default is the minimum valid number possible in c++.  |
| virtual | **[~PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-~pathsection)**()<br>Virtual destructor for the abstract class.  |

**Protected Functions inherited from [PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/)**

|                | Name           |
| -------------- | -------------- |
| | **[PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-pathsection)**(bool can_be_composed)<br>Constructor for the abstract class i NOTE: this class is virtual, therefore an object of type [PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/) cannot be instantiated in memory. Only derivatives of the class [PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/).  |
| bool | **[setMaxGammaValue](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-setmaxgammavalue)**(double gamma_max)<br>Method to update the max of the gamma parameter Validates if the value received is greater than gamma_min.  |
| bool | **[setMinGammaValue](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-setmingammavalue)**(double gamma_min)<br>Method to update the min value of the gamma parameter Validades if the value is received is smaller than gamma_max.  |
| double | **[GradientDescent](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-gradientdescent)**(double gamma_o, Eigen::Vector3d & x_pos, double tolerance)<br>Method that implements the gradient descent to minimize the error of ||pd(gamma) - p_vehicle||.  |
| double | **[getInitialGammaEstimate](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-getinitialgammaestimate)**(Eigen::Vector3d & x_pos, int num_partitions, double min_val, double max_val)<br>Method to get an initial estimate for the gamma. It divides the section into n sections and search for local minimums in the function that computes the distance of the vehicle inside those sections.  |
| double | **[bisection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-bisection)**(Eigen::Vector3d & x_pos, double a, double b)<br>Method to implement bisection method  |


## Detailed Description

```cpp
class Circle2D;
```

Class that implements a 2D Circle section. 

**Author**: 

  * Marcelo Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 

This class is used as a part of the sections library 

## Public Functions Documentation

### function Circle2D

```cpp
Circle2D(
    double radius,
    double center_x,
    double center_y
)
```

Constructor for the path section that assumes the plane in which the path is placed is z=0. 

**Parameters**: 

  * **radius** The radius in m of the Circle 
  * **center_x** The x coordinate of the center of the circle 
  * **center_y** The y coordinate of the center of the circle 


### function Circle2D

```cpp
Circle2D(
    double radius,
    double center_x,
    double center_y,
    double z
)
```

Constructor for the path section. 

**Parameters**: 

  * **radius** The radius in m of the Circle 
  * **center_x** The x coordinate of the center of the circle 
  * **center_y** The y coordinate of the center of the circle 
  * **z** The z axis where to place the circle 


### function eq_pd

```cpp
virtual Eigen::Vector3d eq_pd(
    double t
) override
```

[Path](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/) Section equation. 

**Parameters**: 

  * **t** The path parameter


**Return**: An Eigen::Vector3d with the values from the equation of the path 

**Reimplements**: [PathSection::eq_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-eq-pd)


### function eq_d_pd

```cpp
virtual Eigen::Vector3d eq_d_pd(
    double t
) override
```

First derivative of the path section equation with respect to the path parameter. 

**Parameters**: 

  * **t** The path parameter


**Return**: An Eigen::Vector3d with the values from the derivative of the equation 

**Reimplements**: [PathSection::eq_d_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-eq-d-pd)


### function eq_dd_pd

```cpp
virtual Eigen::Vector3d eq_dd_pd(
    double t
) override
```

Second derivative of the path section equation with respect to the path parameter t. 

**Parameters**: 

  * **t** The path parameter


**Return**: An Eigen::Vector3d with the values from the second derivative of the equation 

**Reimplements**: [PathSection::eq_dd_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-eq-dd-pd)


### function curvature

```cpp
virtual double curvature(
    double t
) override
```

Compute the curvature using only the radius. Computed as 1/Radius. 

**Parameters**: 

  * **t** The path paramter


**Return**: A double with the curvature of that section (1/radius) 

**Reimplements**: [PathSection::curvature](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-curvature)


### function getClosestPointGamma

```cpp
virtual double getClosestPointGamma(
    Eigen::Vector3d & coordinate
) override
```

Method for getting the gamma of the closest point. 

**Parameters**: 

  * **coordinate** The vehicle position


**Return**: A double with the gamma corresponding to the closest point in the path

**Reimplements**: [PathSection::getClosestPointGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-getclosestpointgamma)



Method to return the closest point to the path By default just calls the Gradient Descent algorithm 


-------------------------------

Updated on 2022-05-30 at 08:04:27 +0000