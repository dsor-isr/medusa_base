---
title: ThrustAllocation
summary: ROS implementation of the thrust allocation. Receives forces applied to the vehicle and calculates the desired forces to each thruster based on the pseudo inverse of the thrust allocation matrix. 

---

# ThrustAllocation



ROS implementation of the thrust allocation. Receives forces applied to the vehicle and calculates the desired forces to each thruster based on the pseudo inverse of the thrust allocation matrix. 


`#include <thruster_allocation.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[ThrustAllocation](/medusa_base/api/markdown/medusa_control/static_thruster_allocation/Classes/classThrustAllocation/#function-thrustallocation)**(ros::NodeHandle & nh)<br>Thrust Allocation class constructor.  |
| void | **[initializeSubscribers](/medusa_base/api/markdown/medusa_control/static_thruster_allocation/Classes/classThrustAllocation/#function-initializesubscribers)**(ros::NodeHandle & nh)<br>Function to initialize subscribers.  |
| void | **[initializePublishers](/medusa_base/api/markdown/medusa_control/static_thruster_allocation/Classes/classThrustAllocation/#function-initializepublishers)**(ros::NodeHandle & nh)<br>Function to initialize publishers.  |
| void | **[loadParams](/medusa_base/api/markdown/medusa_control/static_thruster_allocation/Classes/classThrustAllocation/#function-loadparams)**(ros::NodeHandle & nh)<br>Function to read parameters. Reads the thruster allocation matrix and computes its pseudo inverse.  |
| void | **[saturateVector](/medusa_base/api/markdown/medusa_control/static_thruster_allocation/Classes/classThrustAllocation/#function-saturatevector)**(Eigen::VectorXd & thr_thrust)<br>Given a force vector for each thruster, saturate the norm of the vector based on the maximum force of the thruster.  |
| void | **[readTAM](/medusa_base/api/markdown/medusa_control/static_thruster_allocation/Classes/classThrustAllocation/#function-readtam)**(ros::NodeHandle & nh)<br>Function to read a thruster allocation matrix and compute its pseudo inverse.  |
| void | **[readCT](/medusa_base/api/markdown/medusa_control/static_thruster_allocation/Classes/classThrustAllocation/#function-readct)**(ros::NodeHandle & nh)<br>Function to read the ct parameters (conversion from thrust to RPM and vice versa)  |
| void | **[readRPMGain](/medusa_base/api/markdown/medusa_control/static_thruster_allocation/Classes/classThrustAllocation/#function-readrpmgain)**(ros::NodeHandle & nh)<br>Function to read the actuators gain (RPM max value / 100).  |
| void | **[thrusterAllocation](/medusa_base/api/markdown/medusa_control/static_thruster_allocation/Classes/classThrustAllocation/#function-thrusterallocation)**(const auv_msgs::BodyForceRequest & msg)<br>Callback function of the topic with the tau (force request)  |

## Public Functions Documentation

### function ThrustAllocation

```cpp
ThrustAllocation(
    ros::NodeHandle & nh
)
```

Thrust Allocation class constructor. 

**Parameters**: 

  * **nh** ROS nodehandle to publish, subscribe and read relevant parameters 


### function initializeSubscribers

```cpp
void initializeSubscribers(
    ros::NodeHandle & nh
)
```

Function to initialize subscribers. 

**Parameters**: 

  * **nh** ROS nodehandle to publish, subscribe and read relevant 


### function initializePublishers

```cpp
void initializePublishers(
    ros::NodeHandle & nh
)
```

Function to initialize publishers. 

**Parameters**: 

  * **nh** ROS nodehandle to publish, subscribe and read relevant 


### function loadParams

```cpp
void loadParams(
    ros::NodeHandle & nh
)
```

Function to read parameters. Reads the thruster allocation matrix and computes its pseudo inverse. 

**Parameters**: 

  * **nh** ROS nodehandle to publish, subscribe and read relevant 


### function saturateVector

```cpp
void saturateVector(
    Eigen::VectorXd & thr_thrust
)
```

Given a force vector for each thruster, saturate the norm of the vector based on the maximum force of the thruster. 

### function readTAM

```cpp
void readTAM(
    ros::NodeHandle & nh
)
```

Function to read a thruster allocation matrix and compute its pseudo inverse. 

**Parameters**: 

  * **nh** 


### function readCT

```cpp
void readCT(
    ros::NodeHandle & nh
)
```

Function to read the ct parameters (conversion from thrust to RPM and vice versa) 

**Parameters**: 

  * **nh** 


### function readRPMGain

```cpp
void readRPMGain(
    ros::NodeHandle & nh
)
```

Function to read the actuators gain (RPM max value / 100). 

**Parameters**: 

  * **nh** 


### function thrusterAllocation

```cpp
void thrusterAllocation(
    const auv_msgs::BodyForceRequest & msg
)
```

Callback function of the topic with the tau (force request) 

**Parameters**: 

  * **msg** Variable containing the force request 


-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000