---
title: EventTriggered
summary: Implements a CPF controller using Event triggered communications. 

---

# EventTriggered



Implements a CPF controller using Event triggered communications.  [More...](#detailed-description)


`#include <EventTriggered.h>`

Inherits from [CPFControl](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[EventTriggered](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/#function-eventtriggered)**(Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic > & adjency_matrix, unsigned int vehicle_ID, double k_epsilon, double c0, double c1, double alpha)<br>Constructor for the [EventTriggered](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/) CPF class. Receive the adjency matrix as a parameter.  |
| | **[~EventTriggered](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/#function-~eventtriggered)**()<br>The destructor for the [EventTriggered](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/) class.  |
| virtual double | **[coordinationController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/#function-coordinationcontroller)**(double time) override<br>Method to that updated the coordination control law and returns the correction speed vc to be used by the virtual target.  |
| virtual bool | **[updateVehiclesInformation](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/#function-updatevehiclesinformation)**(double time, unsigned int ID, double gamma, double vd) override<br>Method to update each individual vehicle information given an int with the ID of the vehicle and the new virtual target value.  |
| virtual bool | **[publishCurrentGamma](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/#function-publishcurrentgamma)**(double time) override<br>Method to inform the user if the current gamma should be published or not This method will update the value of gamma last sent to the network when this method returns true.  |
| virtual bool | **[reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/#function-reset)**() override<br>Method to reset the current CPF controller.  |

## Protected Functions

|                | Name           |
| -------------- | -------------- |
| void | **[predictVehicleEstimator](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/#function-predictvehicleestimator)**(unsigned int ID, double time)<br>Method to estimate the current gamma of a vehicle represented by ID.  |

## Additional inherited members

**Public Functions inherited from [CPFControl](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/)**

|                | Name           |
| -------------- | -------------- |
| Eigen::MatrixXi | **[getAdjencyMatrix](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-getadjencymatrix)**()<br>Method to get the Adjency Matrix that represents the network topology.  |
| Eigen::VectorXi | **[getNeighbors](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-getneighbors)**()<br>Method to get a vector with the neighbors of the current vehicle.  |
| bool | **[updateAdjencyMatrix](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-updateadjencymatrix)**(Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic > & adjency_matrix)<br>Method to update the Adjency Matrix that represents the network topology.  |
| unsigned int | **[getNetworkSize](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-getnetworksize)**()<br>Method to get the number of vehicles used in the network.  |
| unsigned int | **[getCurrentVehicleID](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-getcurrentvehicleid)**()<br>Method to get the current vehicle ID in the network.  |
| virtual | **[~CPFControl](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-~cpfcontrol)**()<br>Virtual destructor for the abstract class.  |

**Protected Functions inherited from [CPFControl](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/)**

|                | Name           |
| -------------- | -------------- |
| | **[CPFControl](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-cpfcontrol)**(Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic > & adjency_matrix, unsigned int vehicle_ID)<br>Constructor for the abstract class. Receive the adjency matrix as a parameter.  |


## Detailed Description

```cpp
class EventTriggered;
```

Implements a CPF controller using Event triggered communications. 

**Author**: 

  * Marcelo Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: GPLv3 
## Public Functions Documentation

### function EventTriggered

```cpp
EventTriggered(
    Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic > & adjency_matrix,
    unsigned int vehicle_ID,
    double k_epsilon,
    double c0,
    double c1,
    double alpha
)
```

Constructor for the [EventTriggered](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/) CPF class. Receive the adjency matrix as a parameter. 

**Parameters**: 

  * **adjency_matrix** An Eigen adjency matrix 
  * **vehicle_ID** An unsigned int with the ID of this particular vehicle 
  * **k_epsilon** The gain for the correction control law 
  * **c0** The gain for the event trigger threshold 
  * **c1** The gain for the event trigger threshold 
  * **alpha** The gian for the event trigger threshold 


### function ~EventTriggered

```cpp
~EventTriggered()
```

The destructor for the [EventTriggered](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/) class. 

Destructor for the [EventTriggered](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classEventTriggered/) class. 


### function coordinationController

```cpp
virtual double coordinationController(
    double time
) override
```

Method to that updated the coordination control law and returns the correction speed vc to be used by the virtual target. 

**Parameters**: 

  * **time** An int with the current time expressed in seconds


**Return**: A double with the speed correction term vc 

**Reimplements**: [CPFControl::coordinationController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-coordinationcontroller)


### function updateVehiclesInformation

```cpp
virtual bool updateVehiclesInformation(
    double time,
    unsigned int ID,
    double gamma,
    double vd
) override
```

Method to update each individual vehicle information given an int with the ID of the vehicle and the new virtual target value. 

**Parameters**: 

  * **time** An int with the current time expressed in seconds 
  * **ID** An int with the ID of the vehicle which to update information 
  * **gamma** A double with the new information of the virtual target position 
  * **vd** A double with the desired velocity of the virtual target of that particular vehicle


**Return**: A boolean with the information whether the information was used with success or not 

**Reimplements**: [CPFControl::updateVehiclesInformation](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-updatevehiclesinformation)


### function publishCurrentGamma

```cpp
virtual bool publishCurrentGamma(
    double time
) override
```

Method to inform the user if the current gamma should be published or not This method will update the value of gamma last sent to the network when this method returns true. 

**Parameters**: 

  * **time** The current time expressed in seconds


**Return**: A boolean that is true if the current gamma should be published 

**Reimplements**: [CPFControl::publishCurrentGamma](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-publishcurrentgamma)


Method to inform the user if the current gamma should be published or not.


### function reset

```cpp
virtual bool reset() override
```

Method to reset the current CPF controller. 

**Return**: A boolean whether it was reset correctly or not 

**Reimplements**: [CPFControl::reset](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/classCPFControl/#function-reset)


## Protected Functions Documentation

### function predictVehicleEstimator

```cpp
void predictVehicleEstimator(
    unsigned int ID,
    double time
)
```

Method to estimate the current gamma of a vehicle represented by ID. 

**Parameters**: 

  * **ID** The ID of the vehicle to update the prediction 
  * **time** The current time expressed in seconds 


-------------------------------

Updated on 2022-05-30 at 08:04:27 +0000