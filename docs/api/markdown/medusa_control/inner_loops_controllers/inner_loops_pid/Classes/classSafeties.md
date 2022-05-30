---
title: Safeties

---

# Safeties





## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[Safeties](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classSafeties/#function-safeties)**(ros::NodeHandle & nh)<br>[Innerloops]() safeties constructor.  |
| virtual | **[~Safeties](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classSafeties/#function-~safeties)**()<br>[Innerloops]() safeties Destructor.  |
| void | **[loadParams](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classSafeties/#function-loadparams)**(ros::NodeHandle & nh)<br>Method to read parameters from yaml files.  |
| void | **[initializeSubscribers](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classSafeties/#function-initializesubscribers)**(ros::NodeHandle & nh)<br>Method to initialize subscribers.  |
| void | **[initializePublishers](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classSafeties/#function-initializepublishers)**(ros::NodeHandle & nh)<br>Method to initialize publishers.  |
| void | **[depthSafetyCallback](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classSafeties/#function-depthsafetycallback)**(const std_msgs::Float64 & msg)<br>Method called when a depth reference is received. Will check safeties regarding altitude: if valid will publish in the depth controller topic the desired reference, otherwise will publish the minimum altitude in the altitude controller.  |
| void | **[altitudeSafetyCallback](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classSafeties/#function-altitudesafetycallback)**(const std_msgs::Float64 & msg)<br>Method called when a altitude reference is received. Will check safeties regarding altitude: if above the minimum altitude will publish the desired reference, otherwise will publish the minimum altitude.  |
| void | **[stateCallback](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classSafeties/#function-statecallback)**(const auv_msgs::NavigationStatus & msg)<br>Method called when a state msg is received. Updates depth and altitude values to calculate the water collumn needed for depth safeties.  |

## Public Functions Documentation

### function Safeties

```cpp
Safeties(
    ros::NodeHandle & nh
)
```

[Innerloops]() safeties constructor. 

**Parameters**: 

  * **nh** ros nodehandle to subscribe and publish topics 


### function ~Safeties

```cpp
virtual ~Safeties()
```

[Innerloops]() safeties Destructor. 

### function loadParams

```cpp
void loadParams(
    ros::NodeHandle & nh
)
```

Method to read parameters from yaml files. 

**Parameters**: 

  * **nh** ros nodehandle to subscribe and publish topics 


### function initializeSubscribers

```cpp
void initializeSubscribers(
    ros::NodeHandle & nh
)
```

Method to initialize subscribers. 

**Parameters**: 

  * **nh** ros nodehandle to subscribe and publish topics 


### function initializePublishers

```cpp
void initializePublishers(
    ros::NodeHandle & nh
)
```

Method to initialize publishers. 

**Parameters**: 

  * **nh** ros nodehandle to subscribe and publish topics 


### function depthSafetyCallback

```cpp
void depthSafetyCallback(
    const std_msgs::Float64 & msg
)
```

Method called when a depth reference is received. Will check safeties regarding altitude: if valid will publish in the depth controller topic the desired reference, otherwise will publish the minimum altitude in the altitude controller. 

**Parameters**: 

  * **msg** Desired depth 


### function altitudeSafetyCallback

```cpp
void altitudeSafetyCallback(
    const std_msgs::Float64 & msg
)
```

Method called when a altitude reference is received. Will check safeties regarding altitude: if above the minimum altitude will publish the desired reference, otherwise will publish the minimum altitude. 

**Parameters**: 

  * **msg** Desired altitude 


### function stateCallback

```cpp
void stateCallback(
    const auv_msgs::NavigationStatus & msg
)
```

Method called when a state msg is received. Updates depth and altitude values to calculate the water collumn needed for depth safeties. 

**Parameters**: 

  * **msg** Vehicles state 


-------------------------------

Updated on 2022-05-30 at 08:04:28 +0000