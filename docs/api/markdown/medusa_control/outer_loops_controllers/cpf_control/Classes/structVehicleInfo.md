---
title: VehicleInfo
summary: Auxiliar structure to hold information regarding one vehicle. 

---

# VehicleInfo



Auxiliar structure to hold information regarding one vehicle.  [More...](#detailed-description)


`#include <EventTriggered.h>`

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| double | **[gamma](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/structVehicleInfo/#variable-gamma)** <br>The real value of gamma received by the network and the corresponding time at which this value was received by the network.  |
| double | **[time](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/structVehicleInfo/#variable-time)** <br>The time instant expressed in seconds corresponding to the instant when gamma was received.  |
| double | **[gamma_hat](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/structVehicleInfo/#variable-gamma-hat)** <br>The estimated value of gamma.  |
| double | **[vd](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/structVehicleInfo/#variable-vd)** <br>The desired speed for that gamma.  |
| bool | **[is_active](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/cpf_control/Classes/structVehicleInfo/#variable-is-active)** <br>Flag that will become true when the first value is received from the network for this vehicle.  |

## Detailed Description

```cpp
struct VehicleInfo;
```

Auxiliar structure to hold information regarding one vehicle. 

**Author**: 

  * Marcelo Jacinto 
  * Joao Quintas 
  * Joao Cruz 
  * Hung Tuan 


**Version**: 1.0a 

**Date**: 2021 

**Copyright**: MIT 
## Public Attributes Documentation

### variable gamma

```cpp
double gamma {0.0};
```

The real value of gamma received by the network and the corresponding time at which this value was received by the network. 

### variable time

```cpp
double time {0.0};
```

The time instant expressed in seconds corresponding to the instant when gamma was received. 

### variable gamma_hat

```cpp
double gamma_hat {0.0};
```

The estimated value of gamma. 

### variable vd

```cpp
double vd {0.0};
```

The desired speed for that gamma. 

### variable is_active

```cpp
bool is_active {false};
```

Flag that will become true when the first value is received from the network for this vehicle. 

-------------------------------

Updated on 2022-05-30 at 08:04:27 +0000