---
title: PathSection
summary: An abstract class that is used as a template for Path Sections. 

---

# PathSection



An abstract class that is used as a template for [Path]() Sections.  [More...](#detailed-description)


`#include <PathSection.h>`

Inherited by [Arc2D](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classArc2D/), [Bernoulli](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classBernoulli/), [Circle2D](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/), [Line](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classLine/), [Polinomial5](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPolinomial5/), [Polynomial5](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPolynomial5/), [Sinusoid2D](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSinusoid2D/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| virtual Eigen::Vector3d | **[eq_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-eq-pd)**(double t) =0<br>The [Path]() section equation.  |
| virtual Eigen::Vector3d | **[eq_d_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-eq-d-pd)**(double t) =0<br>First derivative of the path section equation with respect to path parameter t.  |
| virtual Eigen::Vector3d | **[eq_dd_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-eq-dd-pd)**(double t) =0<br>Second derivative of the path section equation with respect to the path parameter t.  |
| virtual double | **[tangent](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-tangent)**(double t)<br>Default method for computing the tangent to the path section.  |
| virtual double | **[curvature](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-curvature)**(double t)<br>Default method for computing the curvature. The default implementation implements the general formula to compute the curvature based on the derivative equations of the path.  |
| virtual double | **[derivative_norm](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-derivative-norm)**(double t)<br>Default method for computing the norm of the derivative.  |
| virtual double | **[getClosestPointGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-getclosestpointgamma)**(Eigen::Vector3d & coordinate)<br>Default method for getting the gamma of the closed point to the path. By default this method uses gradient Descent algorithm to compute the closest point in the path with the initial guess of gamma=0.0.  |
| bool | **[can_be_composed](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-can-be-composed)**()<br>Method to return whether a pathSection can be composed with other path sections or not.  |
| double | **[limitGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-limitgamma)**(double t)<br>Method to limit the gamma between the minimum and maximum value    By default the maximum gamma is a very high number.  |
| double | **[getMaxGammaValue](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-getmaxgammavalue)**()<br>Method used to get the maximum gamma allowed by the path section By the default is the maximum valid number possible in c++.  |
| double | **[getMinGammaValue](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-getmingammavalue)**()<br>Method used to get the minimum gamma allowed by the path section By default is the minimum valid number possible in c++.  |
| virtual | **[~PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/#function-~pathsection)**()<br>Virtual destructor for the abstract class.  |

## Protected Functions

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
class PathSection;
```

An abstract class that is used as a template for [Path]() Sections. 

**Author**: 

  * Marcelo Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 

For path sections that can_be_composed the minimum value of gamma will be defaulted to 0 (and cannot be changed) in order not to mess with the [Path](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPath/) section switching algorithm. The maximum value can be any greater than 0.

For path sections where can_be_composed = false, the minimum value for gamma if -inf an the maximum value if +inf (but these can be changed) 

## Public Functions Documentation

### function eq_pd

```cpp
virtual Eigen::Vector3d eq_pd(
    double t
) =0
```

The [Path]() section equation. 

**Parameters**: 

  * **t** The path parameter


**Return**: An Eigen::Vector3d with the equation of the path with respect to the path parameter 

**Reimplemented by**: [Polinomial5::eq_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPolinomial5/#function-eq-pd), [Polynomial5::eq_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPolynomial5/#function-eq-pd), [Line::eq_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classLine/#function-eq-pd), [Sinusoid2D::eq_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSinusoid2D/#function-eq-pd), [Circle2D::eq_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/#function-eq-pd), [Bernoulli::eq_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classBernoulli/#function-eq-pd), [Arc2D::eq_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classArc2D/#function-eq-pd)


### function eq_d_pd

```cpp
virtual Eigen::Vector3d eq_d_pd(
    double t
) =0
```

First derivative of the path section equation with respect to path parameter t. 

**Parameters**: 

  * **t** The path parameter


**Return**: An Eigen::Vector3d with the first derivative of the path equation with respect to the path parameter 

**Reimplemented by**: [Polinomial5::eq_d_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPolinomial5/#function-eq-d-pd), [Polynomial5::eq_d_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPolynomial5/#function-eq-d-pd), [Line::eq_d_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classLine/#function-eq-d-pd), [Sinusoid2D::eq_d_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSinusoid2D/#function-eq-d-pd), [Circle2D::eq_d_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/#function-eq-d-pd), [Bernoulli::eq_d_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classBernoulli/#function-eq-d-pd), [Arc2D::eq_d_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classArc2D/#function-eq-d-pd)


### function eq_dd_pd

```cpp
virtual Eigen::Vector3d eq_dd_pd(
    double t
) =0
```

Second derivative of the path section equation with respect to the path parameter t. 

**Parameters**: 

  * **t** The path parameter


**Return**: An Eigen::Vector3d with the second derivative of the path equation with respect to the path paramter 

**Reimplemented by**: [Polinomial5::eq_dd_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPolinomial5/#function-eq-dd-pd), [Polynomial5::eq_dd_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPolynomial5/#function-eq-dd-pd), [Line::eq_dd_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classLine/#function-eq-dd-pd), [Circle2D::eq_dd_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/#function-eq-dd-pd), [Bernoulli::eq_dd_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classBernoulli/#function-eq-dd-pd), [Sinusoid2D::eq_dd_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSinusoid2D/#function-eq-dd-pd), [Arc2D::eq_dd_pd](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classArc2D/#function-eq-dd-pd)


### function tangent

```cpp
virtual double tangent(
    double t
)
```

Default method for computing the tangent to the path section. 

**Parameters**: 

  * **t** The path parameter


**Return**: A double with the angle of the tangent to the path 

### function curvature

```cpp
virtual double curvature(
    double t
)
```

Default method for computing the curvature. The default implementation implements the general formula to compute the curvature based on the derivative equations of the path. 

**Parameters**: 

  * **t** The path parameter


**Return**: A double with the path curvature 

**Reimplemented by**: [Line::curvature](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classLine/#function-curvature), [Circle2D::curvature](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/#function-curvature), [Bernoulli::curvature](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classBernoulli/#function-curvature), [Arc2D::curvature](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classArc2D/#function-curvature)


### function derivative_norm

```cpp
virtual double derivative_norm(
    double t
)
```

Default method for computing the norm of the derivative. 

**Parameters**: 

  * **t** The path parameter


**Return**: A double with the norm of the derivative of the path position pd 

### function getClosestPointGamma

```cpp
virtual double getClosestPointGamma(
    Eigen::Vector3d & coordinate
)
```

Default method for getting the gamma of the closed point to the path. By default this method uses gradient Descent algorithm to compute the closest point in the path with the initial guess of gamma=0.0. 

**Parameters**: 

  * **coordinate** A Eigen::Vector3d with the coordinates of the vehicle


**Return**: A double with the closest point in the path

**Reimplemented by**: [Polinomial5::getClosestPointGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPolinomial5/#function-getclosestpointgamma), [Polynomial5::getClosestPointGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPolynomial5/#function-getclosestpointgamma), [Sinusoid2D::getClosestPointGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classSinusoid2D/#function-getclosestpointgamma), [Line::getClosestPointGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classLine/#function-getclosestpointgamma), [Circle2D::getClosestPointGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classCircle2D/#function-getclosestpointgamma), [Bernoulli::getClosestPointGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classBernoulli/#function-getclosestpointgamma), [Arc2D::getClosestPointGamma](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classArc2D/#function-getclosestpointgamma)


This is not the most efficient way to compute the closest point for several kinds of paths, but it is the most general, hence used as the default one


Method to return the closest point to the path By default just calls the Gradient Descent algorithm 


### function can_be_composed

```cpp
bool can_be_composed()
```

Method to return whether a pathSection can be composed with other path sections or not. 

**Return**: A boolean indicating wether the pathsection can be composed with other sections or not 

### function limitGamma

```cpp
double limitGamma(
    double t
)
```

Method to limit the gamma between the minimum and maximum value    By default the maximum gamma is a very high number. 

**Parameters**: 

  * **t** The path parameter also known as gamma


**Return**: A double with the path parameter limited between the valid bounds 

### function getMaxGammaValue

```cpp
double getMaxGammaValue()
```

Method used to get the maximum gamma allowed by the path section By the default is the maximum valid number possible in c++. 

**Return**: A double with the maximum value that can be achieved with gamma 

### function getMinGammaValue

```cpp
double getMinGammaValue()
```

Method used to get the minimum gamma allowed by the path section By default is the minimum valid number possible in c++. 

**Return**: A double with the minimum value the can be achieved with gamma 

### function ~PathSection

```cpp
virtual ~PathSection()
```

Virtual destructor for the abstract class. 

## Protected Functions Documentation

### function PathSection

```cpp
PathSection(
    bool can_be_composed
)
```

Constructor for the abstract class i NOTE: this class is virtual, therefore an object of type [PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/) cannot be instantiated in memory. Only derivatives of the class [PathSection](/medusa_base/api/markdown/medusa_planning/dsor_paths/Classes/classPathSection/). 

**Parameters**: 

  * **can_be_composed** A boolean that indicated that if this kind of path can be composed with other path sections or not 


### function setMaxGammaValue

```cpp
bool setMaxGammaValue(
    double gamma_max
)
```

Method to update the max of the gamma parameter Validates if the value received is greater than gamma_min. 

**Parameters**: 

  * **gamma_max** The desired max value for gamma


**Return**: True if new value was accepted 

### function setMinGammaValue

```cpp
bool setMinGammaValue(
    double gamma_min
)
```

Method to update the min value of the gamma parameter Validades if the value is received is smaller than gamma_max. 

**Parameters**: 

  * **gamma_min** The desired min value for gamma


**Return**: True if the new value was accepted


NOTE: for paths where can_be_composed == true, this function always returns false as it is required for those kinds of segments to start with 0 (but no limit is put on gamma max) 


### function GradientDescent

```cpp
double GradientDescent(
    double gamma_o,
    Eigen::Vector3d & x_pos,
    double tolerance
)
```

Method that implements the gradient descent to minimize the error of ||pd(gamma) - p_vehicle||. 

**Parameters**: 

  * **gamma_o** The initial guess for the path paramter 
  * **x_pos** The position of the vehicle 
  * **tolerance** A constant tweaking parameter


**Return**: the estimated gamma value that minimizes the error 

### function getInitialGammaEstimate

```cpp
double getInitialGammaEstimate(
    Eigen::Vector3d & x_pos,
    int num_partitions,
    double min_val,
    double max_val
)
```

Method to get an initial estimate for the gamma. It divides the section into n sections and search for local minimums in the function that computes the distance of the vehicle inside those sections. 

**Parameters**: 

  * **x_pos** The position of the vehicle 
  * **num_partitions** The number of divisions to make on the path 
  * **min_val** The minimum boundary to search in gamma 
  * **max_val** The maximum boundary to search in gamma


**Return**: A double with a good gamma to use as an initialization for the estimation 

Then grabs the gamma that minimizes the most from all sections


### function bisection

```cpp
double bisection(
    Eigen::Vector3d & x_pos,
    double a,
    double b
)
```

Method to implement bisection method 

**Parameters**: 

  * **x_pos** The position of the vehicle 
  * **a** The left bound for the gamma value 
  * **b** The right bound for the gamma value


**Return**: The most likely value of gamma 

-------------------------------

Updated on 2022-05-30 at 08:04:27 +0000