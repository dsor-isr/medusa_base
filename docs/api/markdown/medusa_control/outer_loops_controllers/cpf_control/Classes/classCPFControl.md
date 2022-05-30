---
title: CPFControl
summary: Abstract class to implement the cooperative path following synchronization controller. 

---

# CPFControl



Abstract class to implement the cooperative path following synchronization controller.  [More...](#detailed-description)


`#include <CPFControl.h>`

Inherited by [EventTriggered](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| virtual double | **[coordinationController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-coordinationcontroller)**(double time) =0<br>Method to that updated the coordination control law and returns the correction speed vc to be used by the virtual target.  |
| virtual bool | **[updateVehiclesInformation](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-updatevehiclesinformation)**(double time, unsigned int ID, double gamma, double vd) =0<br>Method to update each individual vehicle information given an int with the ID of the vehicle and the new virtual target value.  |
| virtual bool | **[publishCurrentGamma](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-publishcurrentgamma)**(double time) =0<br>Method to inform the user if the current gamma should be published or not.  |
| virtual bool | **[reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-reset)**() =0<br>Method to reset the current CPF controller.  |
| Eigen::MatrixXi | **[getAdjencyMatrix](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-getadjencymatrix)**()<br>Method to get the Adjency Matrix that represents the network topology.  |
| Eigen::VectorXi | **[getNeighbors](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-getneighbors)**()<br>Method to get a vector with the neighbors of the current vehicle.  |
| bool | **[updateAdjencyMatrix](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-updateadjencymatrix)**(Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic > & adjency_matrix)<br>Method to update the Adjency Matrix that represents the network topology.  |
| unsigned int | **[getNetworkSize](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-getnetworksize)**()<br>Method to get the number of vehicles used in the network.  |
| unsigned int | **[getCurrentVehicleID](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-getcurrentvehicleid)**()<br>Method to get the current vehicle ID in the network.  |
| virtual | **[~CPFControl](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-~cpfcontrol)**()<br>Virtual destructor for the abstract class.  |

## Protected Functions

|                | Name           |
| -------------- | -------------- |
| | **[CPFControl](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-cpfcontrol)**(Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic > & adjency_matrix, unsigned int vehicle_ID)<br>Constructor for the abstract class. Receive the adjency matrix as a parameter.  |

## Detailed Description

```cpp
class CPFControl;
```

Abstract class to implement the cooperative path following synchronization controller. 

**Author**: 

  * Marcelo Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 

In this class it is assumed that each vehicle has an ID which corresponds to each position in the adjency matrix that describes the topology of the network. The first ID is assumed to be 0. 

## Public Functions Documentation

### function coordinationController

```cpp
virtual double coordinationController(
    double time
) =0
```

Method to that updated the coordination control law and returns the correction speed vc to be used by the virtual target. 

**Parameters**: 

  * **time** An int with the current time expressed in seconds


**Return**: A double with the speed correction term vc 

**Reimplemented by**: [EventTriggered::coordinationController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/#function-coordinationcontroller)


### function updateVehiclesInformation

```cpp
virtual bool updateVehiclesInformation(
    double time,
    unsigned int ID,
    double gamma,
    double vd
) =0
```

Method to update each individual vehicle information given an int with the ID of the vehicle and the new virtual target value. 

**Parameters**: 

  * **time** An int with the current time expressed in seconds 
  * **ID** An int with the ID of the vehicle which to update information 
  * **gamma** A double with the new information of the virtual target position 
  * **vd** A double with the desired velocity of the virtual target of that particular vehicle


**Return**: A boolean with the information whether the information was used with success or not 

**Reimplemented by**: [EventTriggered::updateVehiclesInformation](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/#function-updatevehiclesinformation)


### function publishCurrentGamma

```cpp
virtual bool publishCurrentGamma(
    double time
) =0
```

Method to inform the user if the current gamma should be published or not. 

**Parameters**: 

  * **time** The current time expressed in seconds


**Return**: A boolean that is true if the current gamma should be published 

**Reimplemented by**: [EventTriggered::publishCurrentGamma](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/#function-publishcurrentgamma)


### function reset

```cpp
virtual bool reset() =0
```

Method to reset the current CPF controller. 

**Return**: A boolean whether it was reset correctly or not 

**Reimplemented by**: [EventTriggered::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/#function-reset)


### function getAdjencyMatrix

```cpp
Eigen::MatrixXi getAdjencyMatrix()
```

Method to get the Adjency Matrix that represents the network topology. 

**Return**: 

  * An eigen adjecy matrix of ints
  * An eigen adjecy matrix of doubles 


### function getNeighbors

```cpp
Eigen::VectorXi getNeighbors()
```

Method to get a vector with the neighbors of the current vehicle. 

**Return**: An eigen vector with ints 

### function updateAdjencyMatrix

```cpp
bool updateAdjencyMatrix(
    Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic > & adjency_matrix
)
```

Method to update the Adjency Matrix that represents the network topology. 

**Return**: A boolean informing if update was done successfully or not 

### function getNetworkSize

```cpp
unsigned int getNetworkSize()
```

Method to get the number of vehicles used in the network. 

**Return**: An int with the number of vehicles in the network 

### function getCurrentVehicleID

```cpp
unsigned int getCurrentVehicleID()
```

Method to get the current vehicle ID in the network. 

**Return**: An int with the current vehicle ID 

### function ~CPFControl

```cpp
virtual ~CPFControl()
```

Virtual destructor for the abstract class. 

## Protected Functions Documentation

### function CPFControl

```cpp
CPFControl(
    Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic > & adjency_matrix,
    unsigned int vehicle_ID
)
```

Constructor for the abstract class. Receive the adjency matrix as a parameter. 

**Parameters**: 

  * **adjency_matrix** An Eigen adjency matrix 
  * **vehicle_ID** The current vehicle ID 


-------------------------------

Updated on 2022-05-30 at 08:04:27 +0000