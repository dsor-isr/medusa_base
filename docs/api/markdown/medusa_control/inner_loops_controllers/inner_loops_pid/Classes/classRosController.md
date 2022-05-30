---
title: RosController
summary: ROS implementation of the innerloops controllers. Based on a desired reference computes the force or torque to be applied. 

---

# RosController



ROS implementation of the innerloops controllers. Based on a desired reference computes the force or torque to be applied. 


`#include <ros_controller.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[RosController](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#function-roscontroller)**(ros::NodeHandle & nh, std::string controller_name, std::string refCallback_topic, double * state, double * force_or_torque, double frequency)<br>Constructor of a innerloop controller.  |
| | **[RosController](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#function-roscontroller)**(ros::NodeHandle & nh, std::string controller_name, std::string refCallback_topic, double * state, double * state_dot, double * force_or_torque, double frequency)<br>Constructor of a innerloop controller.  |
| virtual double | **[computeCommand](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#function-computecommand)**()<br>Core function. Computes the PID output.  |
| void | **[setCircularUnits](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#function-setcircularunits)**(const bool & flag)<br>Setter function for the circular units flag.  |
| void | **[setPositiveOutput](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#function-setpositiveoutput)**(const bool & flag)<br>Set the Positive Output object.  |
| std::string | **[getControllerName](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#function-getcontrollername)**() const<br>Get the Controller Name object.  |
| void | **[setFFGainsPID](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#function-setffgainspid)**(const float & kff, const float & kff_d, const float & kff_lin_drag, const float kff_quad_drag)<br>Set the Feedforwar gains P I D object.  |
| void | **[setGainsPID](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#function-setgainspid)**(const float & kp, const float & ki, const float & kd)<br>Set the Gains P I D object.  |
| void | **[setLimitBoundsPID](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#function-setlimitboundspid)**(const float & max_out, const float & min_out) |

## Protected Functions

|                | Name           |
| -------------- | -------------- |
| void | **[refCallback](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#function-refcallback)**(const std_msgs::Float64 & ptr)<br>Callback function. Saturates the value if boundaries exist.  |
| void | **[init](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#function-init)**(ros::NodeHandle & nh, std::string controller_name, std::string refCallback_topic)<br>Initialize function. Reads the parameters, creates a pid controller if any gain is different from zero and subscribes to the relevant topic.  |
| virtual bool | **[validRef](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#function-validref)**()<br>Check if the reference is new.  |

## Protected Attributes

|                | Name           |
| -------------- | -------------- |
| ros::Duration | **[timeout_ref_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-timeout-ref-)**  |
| std::string | **[controller_name_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-controller-name-)**  |
| double | **[ref_value_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-ref-value-)**  |
| double | **[max_ref_value_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-max-ref-value-)**  |
| double | **[min_ref_value_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-min-ref-value-)**  |
| ros::Time | **[ref_time_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-ref-time-)**  |
| ros::Time | **[last_cmd_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-last-cmd-)**  |
| bool | **[debug_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-debug-)**  |
| medusa_msgs::mPidDebug | **[debug_msg_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-debug-msg-)**  |
| double * | **[state_ptr_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-state-ptr-)**  |
| double * | **[force_or_torque_ptr_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-force-or-torque-ptr-)**  |
| double | **[frequency_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-frequency-)**  |
| bool | **[circular_units_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-circular-units-)**  |
| bool | **[positive_output_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-positive-output-)**  |
| [PID_Controller](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/) * | **[pid_c_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-pid-c-)**  |
| ros::Subscriber | **[ros_sub_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-ros-sub-)**  |
| ros::Publisher | **[debug_pub_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classRosController/#variable-debug-pub-)**  |

## Public Functions Documentation

### function RosController

```cpp
RosController(
    ros::NodeHandle & nh,
    std::string controller_name,
    std::string refCallback_topic,
    double * state,
    double * force_or_torque,
    double frequency
)
```

Constructor of a innerloop controller. 

**Parameters**: 

  * **nh** ROS nodehandle to read parameters and subscribe to relevant topics 
  * **controller_name** Controller name (variable being controlled) 
  * **refCallback_topic** Topic name 
  * **state** Pointer to state variable being controlled 
  * **force_or_torque** Pointer to force or torque output 
  * **frequency** Frequency of controller sampling rate 


### function RosController

```cpp
RosController(
    ros::NodeHandle & nh,
    std::string controller_name,
    std::string refCallback_topic,
    double * state,
    double * state_dot,
    double * force_or_torque,
    double frequency
)
```

Constructor of a innerloop controller. 

**Parameters**: 

  * **nh** ROS nodehandle to read parameters and subscribe to relevant topics 
  * **controller_name** Controller name (variable being controlled) 
  * **refCallback_topic** Topic name 
  * **state** Pointer to state variable being controlled 
  * **state_dot** Pointer to the derivative of the state variable being controlled 
  * **force_or_torque** Pointer to force or torque output 
  * **frequency** Frequency of controller sampling rate 


### function computeCommand

```cpp
virtual double computeCommand()
```

Core function. Computes the PID output. 

**Return**: The force or torque that result from the PID computation 

### function setCircularUnits

```cpp
inline void setCircularUnits(
    const bool & flag
)
```

Setter function for the circular units flag. 

**Parameters**: 

  * **flag** true for controllers using angles, false otherwise 


### function setPositiveOutput

```cpp
inline void setPositiveOutput(
    const bool & flag
)
```

Set the Positive Output object. 

**Parameters**: 

  * **flag** 


### function getControllerName

```cpp
inline std::string getControllerName() const
```

Get the Controller Name object. 

### function setFFGainsPID

```cpp
inline void setFFGainsPID(
    const float & kff,
    const float & kff_d,
    const float & kff_lin_drag,
    const float kff_quad_drag
)
```

Set the Feedforwar gains P I D object. 

**Parameters**: 

  * **kff** Feedforward gain (quadratic) 
  * **kff_d** Feedforwad derivative gain 
  * **kff_lin_drag** Feedforward linear drag gain 
  * **kff_quad_drag** Feedforwad quadratic drag gain 


### function setGainsPID

```cpp
inline void setGainsPID(
    const float & kp,
    const float & ki,
    const float & kd
)
```

Set the Gains P I D object. 

**Parameters**: 

  * **kp** Proportional gain 
  * **ki** Integral gain 
  * **kd** Derivative gain 


### function setLimitBoundsPID

```cpp
inline void setLimitBoundsPID(
    const float & max_out,
    const float & min_out
)
```


## Protected Functions Documentation

### function refCallback

```cpp
void refCallback(
    const std_msgs::Float64 & ptr
)
```

Callback function. Saturates the value if boundaries exist. 

**Parameters**: 

  * **msg** Float64 value of the variable being controlled 


### function init

```cpp
void init(
    ros::NodeHandle & nh,
    std::string controller_name,
    std::string refCallback_topic
)
```

Initialize function. Reads the parameters, creates a pid controller if any gain is different from zero and subscribes to the relevant topic. 

**Parameters**: 

  * **nh** Nodehandle to read parameters and subscribe to relevant topics 
  * **controller_name** Controller name (variable being controlled) 
  * **refCallback_topic** Topic name 


### function validRef

```cpp
virtual bool validRef()
```

Check if the reference is new. 

**Return**: True if valid, false otherwise 

## Protected Attributes Documentation

### variable timeout_ref_

```cpp
ros::Duration timeout_ref_;
```


### variable controller_name_

```cpp
std::string controller_name_;
```


### variable ref_value_

```cpp
double ref_value_;
```


### variable max_ref_value_

```cpp
double max_ref_value_;
```


### variable min_ref_value_

```cpp
double min_ref_value_;
```


### variable ref_time_

```cpp
ros::Time ref_time_;
```


### variable last_cmd_

```cpp
ros::Time last_cmd_;
```


### variable debug_

```cpp
bool debug_;
```


### variable debug_msg_

```cpp
medusa_msgs::mPidDebug debug_msg_;
```


### variable state_ptr_

```cpp
double * state_ptr_;
```


### variable force_or_torque_ptr_

```cpp
double * force_or_torque_ptr_;
```


### variable frequency_

```cpp
double frequency_;
```


### variable circular_units_

```cpp
bool circular_units_;
```


### variable positive_output_

```cpp
bool positive_output_;
```


### variable pid_c_

```cpp
PID_Controller * pid_c_;
```


### variable ros_sub_

```cpp
ros::Subscriber ros_sub_;
```


### variable debug_pub_

```cpp
ros::Publisher debug_pub_;
```


-------------------------------

Updated on 2022-05-30 at 08:04:28 +0000