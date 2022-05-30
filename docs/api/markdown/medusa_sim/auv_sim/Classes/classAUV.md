---
title: AUV
summary: AUV class - implements a Matlab-like simulation of an AUV in C++. 

---

# AUV



[AUV]() class - implements a Matlab-like simulation of an [AUV]() in C++.  [More...](#detailed-description)


`#include <AUV.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[AUV](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/classAUV/#function-auv)**(double mass, double fluid_density, double zg, double vehicle_density, const Eigen::Vector3d & inertia_tensor, const Eigen::Matrix< double, 6, 1 > & linear_damping_tensor, const Eigen::Matrix< double, 6, 1 > & quadratic_damping_tensor, const Eigen::Matrix< double, 6, 1 > & added_mass_tensor, const Eigen::MatrixXd & allocation_matrix, const Eigen::Vector3d & lump_param_positive, const Eigen::Vector3d & lump_param_negative, const Eigen::Vector2d & min_max_thruster_input, double thrusters_gain, double thrusters_pole, double thrusters_delay, double sampling_period, const Eigen::Vector3d & disturbance_mean, const Eigen::Vector3d & disturbance_sigma, const Eigen::Vector3d & disturbance_minimum, const Eigen::Vector3d & disturbance_maximum) |
| void | **[update](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/classAUV/#function-update)**(double dt, const Eigen::VectorXd & thrust) |
| [State](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/structState/) | **[getState](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/classAUV/#function-getstate)**() |
| void | **[setState](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/classAUV/#function-setstate)**(const [State](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/structState/) & state) |
| unsigned int | **[getNumberThrusters](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/classAUV/#function-getnumberthrusters)**() |

## Detailed Description

```cpp
class AUV;
```

[AUV]() class - implements a Matlab-like simulation of an [AUV]() in C++. 

**Author**: Marcelo Jacinto 

**Version**: 1.0.0 

**Date**: 2021/11/12 

**Copyright**: MIT 
## Public Functions Documentation

### function AUV

```cpp
AUV(
    double mass,
    double fluid_density,
    double zg,
    double vehicle_density,
    const Eigen::Vector3d & inertia_tensor,
    const Eigen::Matrix< double, 6, 1 > & linear_damping_tensor,
    const Eigen::Matrix< double, 6, 1 > & quadratic_damping_tensor,
    const Eigen::Matrix< double, 6, 1 > & added_mass_tensor,
    const Eigen::MatrixXd & allocation_matrix,
    const Eigen::Vector3d & lump_param_positive,
    const Eigen::Vector3d & lump_param_negative,
    const Eigen::Vector2d & min_max_thruster_input,
    double thrusters_gain,
    double thrusters_pole,
    double thrusters_delay,
    double sampling_period,
    const Eigen::Vector3d & disturbance_mean,
    const Eigen::Vector3d & disturbance_sigma,
    const Eigen::Vector3d & disturbance_minimum,
    const Eigen::Vector3d & disturbance_maximum
)
```


**Parameters**: 

  * **mass** The mass of the vehicle in Kg 
  * **fluid_density** The density of the fluid the vehicle is in (a.k.a water density) 
  * **zg** The center of gravity of the (sphere-like) vehicle 
  * **vehicle_density** The density of the vehicle (Kg/m^3) 
  * **inertia_tensor** A vector of 3 elements with the diagonal of the inertia matrix 
  * **linear_damping_tensor** A vector of 6 elements with the diagonal of the linear damping matrix 
  * **quadratic_damping_tensor** A vector of 6 elements with the diagonal of the quadratic damping matrix 
  * **added_mass_tensor** A vector of 6 elements with the diagonal of the added mass matrix 
  * **allocation_matrix** A matrix with the contributions of each thruster to the forces in X,Y and Z and the arms for computing the moments of inertia later [Fx, Fy, Fz, lx, ly, lz] (each line represents one thruster) 
  * **lump_param_positive** The thrust curve parameters for the right side of the curve 
  * **lump_param_negative** The thrust curve parameters for the left side of the curve 
  * **min_max_thruster_input** The minimum and maximum normalized thruster inputs [min_input, max_input]^T 
  * **thrusters_gain** 
  * **thrusters_pole** 
  * **thrusters_delay** 
  * **sampling_period** An approximated sampling period (s) at which the simulation will run (NOTE: this is needed to discretize the thrusters model only and the integration of the dynamics of the vehicle will use the dt variable provided through the update method) 
  * **disturbance_mean** A vector with the mean of the ocean disturbances (gaussian process) 
  * **disturbance_sigma** A vector with the standard deviation of the ocean disturbances (gaussian process) 
  * **disturbance_minimum** A vector with the minimum values for the ocean disturbances 
  * **disturbance_maximum** A vector with the maximum values for the ocean disturbances 


MedusaAUV class constructor 


### function update

```cpp
void update(
    double dt,
    const Eigen::VectorXd & thrust
)
```


**Parameters**: 

  * **dt** The time difference (in seconds) between the last function call and the disturbance function call 
  * **thrust** A vector of n elements with the thrust to apply to each of the n vehicle thursters (normalized between 0 and 1) 


Method to update the state of the vehicle given the thrust applied to each individual thruster. This method verifies if (dt >= 0) and the size of thrust vector is the same as the number of thrusters of the model. If some of these conditions does not verify, a std::invalid_argument exception is thrown


### function getState

```cpp
inline State getState()
```


**Return**: A state object with the state of the vehicle 

Method that returns a copy of the disturbance state of the vehicle 


### function setState

```cpp
inline void setState(
    const State & state
)
```


**Parameters**: 

  * **state** A state reference that contains the desired state of the vehicle 


Method that sets the disturbance state of the vehicle to a new pre-defined state 


### function getNumberThrusters

```cpp
inline unsigned int getNumberThrusters()
```


**Return**: The number of thrusters of the [AUV](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/classAUV/)

Method that returns the number of thrusters of the [AUV](/medusa_base/api/markdown/medusa_sim/auv_sim/Classes/classAUV/) based on the number of lines of the allocation matrix received by the constructor upon object construction 


-------------------------------

Updated on 2022-05-30 at 08:04:25 +0000