---
title: PID_Controller
summary: Implementation of a PID with anti windup. 

---

# PID_Controller



Implementation of a PID with anti windup. 


`#include <pid_controller.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[PID_Controller](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-pid-controller)**()<br>Constructor of a pid controller with kp, ki and kd equal to 0.  |
| | **[PID_Controller](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-pid-controller)**(float Kp, float Ki, float Kd, float max_error, float max_out)<br>Constructor of a PID controller. Initially enabled.  |
| | **[PID_Controller](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-pid-controller)**(float Kp, float Ki, float Kd, float max_error, float max_out, float min_error, float min_out)<br>Constructor of a PID controller. Initially enabled.  |
| | **[PID_Controller](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-pid-controller)**(float Kp, float Ki, float Kd, float Kff, float Kff_d, float Kff_lin_drag, float Kff_quad_drag, float max_error, float max_out, float min_error, float min_out)<br>Constructor of a PID controller (without low pass filter). Initially enabled.  |
| | **[PID_Controller](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-pid-controller)**(float Kp, float Ki, float Kd, float Kff, float Kff_d, float Kff_lin_drag, float Kff_quad_drag, float max_error, float max_out, float min_error, float min_out, double lpf_dt, double lpf_fc)<br>Constructor of a PID controller (with low pass filter). Initially enabled.  |
| float | **[computeCommand](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-computecommand)**(float error_p, float ref_value, float duration, bool debug)<br>Core function. Computes the output of the PID.  |
| void | **[reset](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-reset)**()<br>Reset function. Sets the integral error term to 0.  |
| void | **[setFFGains](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-setffgains)**(const float & ff_gain, const float & ff_d_gain, const float & ff_lin_drag_gain, const float & ff_quad_drag_gain)<br>Set the feedfoward Gains object.  |
| void | **[setGains](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-setgains)**(const float & kp, const float & ki, const float & kd)<br>Set the Gains object.  |
| void | **[setLimitBounds](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-setlimitbounds)**(const float & max_out, const float & min_out)<br>Set the Limit Bounds object.  |
| std::vector< double > | **[getGains](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-getgains)**() const<br>Get the Gains object.  |
| std::vector< double > | **[getLimitBounds](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-getlimitbounds)**() const<br>Get the Limit Bounds object.  |
| medusa_msgs::mPidDebug | **[getDebugInfo](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#function-getdebuginfo)**() const<br>Get debug info from the PID controller internal variables.  |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| bool | **[disable](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-disable)**  |

## Protected Attributes

|                | Name           |
| -------------- | -------------- |
| float | **[p_gain_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-p-gain-)**  |
| float | **[i_gain_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-i-gain-)**  |
| float | **[d_gain_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-d-gain-)**  |
| float | **[ff_gain_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-ff-gain-)**  |
| float | **[ff_d_gain_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-ff-d-gain-)**  |
| float | **[ff_lin_drag_gain_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-ff-lin-drag-gain-)**  |
| float | **[ff_quad_drag_gain_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-ff-quad-drag-gain-)**  |
| float | **[max_error_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-max-error-)**  |
| float | **[max_out_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-max-out-)**  |
| float | **[min_error_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-min-error-)**  |
| float | **[min_out_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-min-out-)**  |
| float | **[integral_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-integral-)**  |
| float | **[pre_error_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-pre-error-)**  |
| float | **[prev_ref_value_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-prev-ref-value-)**  |
| bool | **[has_lpf_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-has-lpf-)**  |
| std::unique_ptr< LowPassFilter > | **[lpf_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-lpf-)**  |
| medusa_msgs::mPidDebug | **[msg_debug_](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/inner_loops_pid/Classes/classPID__Controller/#variable-msg-debug-)**  |

## Public Functions Documentation

### function PID_Controller

```cpp
PID_Controller()
```

Constructor of a pid controller with kp, ki and kd equal to 0. 

### function PID_Controller

```cpp
PID_Controller(
    float Kp,
    float Ki,
    float Kd,
    float max_error,
    float max_out
)
```

Constructor of a PID controller. Initially enabled. 

**Parameters**: 

  * **Kp** Proporcional gain 
  * **Ki** Integral gain 
  * **Kd** Derivative gain 
  * **max_error** maximum reference error allowed 
  * **max_out** minimum reference error allowed 


### function PID_Controller

```cpp
PID_Controller(
    float Kp,
    float Ki,
    float Kd,
    float max_error,
    float max_out,
    float min_error,
    float min_out
)
```

Constructor of a PID controller. Initially enabled. 

**Parameters**: 

  * **Kp** Proporcional gain 
  * **Ki** Integral gain 
  * **Kd** Derivative gain 
  * **max_error** maximum reference error allowed 
  * **max_out** maximum output allowed 
  * **min_error** minimum refrence error allowed 
  * **min_out** minimum output allowed 


### function PID_Controller

```cpp
PID_Controller(
    float Kp,
    float Ki,
    float Kd,
    float Kff,
    float Kff_d,
    float Kff_lin_drag,
    float Kff_quad_drag,
    float max_error,
    float max_out,
    float min_error,
    float min_out
)
```

Constructor of a PID controller (without low pass filter). Initially enabled. 

**Parameters**: 

  * **Kp** Proporcional gain 
  * **Ki** Integral gain 
  * **Kd** Derivative gain 
  * **Kff** Feedforward gain 
  * **Kff_d** Feedforward gain (linear drag) [] 
  * **Kff_dd** Feedforward gain (quadratic drag) 
  * **max_error** maximum reference error allowed 
  * **max_out** maximum output allowed 
  * **min_error** minimum refrence error allowed 
  * **min_out** minimum output allowed 


### function PID_Controller

```cpp
PID_Controller(
    float Kp,
    float Ki,
    float Kd,
    float Kff,
    float Kff_d,
    float Kff_lin_drag,
    float Kff_quad_drag,
    float max_error,
    float max_out,
    float min_error,
    float min_out,
    double lpf_dt,
    double lpf_fc
)
```

Constructor of a PID controller (with low pass filter). Initially enabled. 

**Parameters**: 

  * **Kp** Proporcional gain 
  * **Ki** Integral gain 
  * **Kd** Derivative gain 
  * **Kff** Feedforward gain 
  * **Kff_d** Feedforward gain (linear drag) [] 
  * **Kff_dd** Feedforward gain (quadratic drag) 
  * **max_error** maximum reference error allowed 
  * **max_out** maximum output allowed 
  * **min_error** minimum refrence error allowed 
  * **min_out** minimum output allowed 
  * **lpf_dt** Low pass filter sampling time 
  * **lpf_fc** Low pass filter cutoff param 


### function computeCommand

```cpp
float computeCommand(
    float error_p,
    float ref_value,
    float duration,
    bool debug
)
```

Core function. Computes the output of the PID. 

**Parameters**: 

  * **error_p** Error between the reference and the estimated variable 
  * **ref_value** Reference value to compute the feedforward term 
  * **duration** Sample time 
  * **debug** Check if we want to generate a debugging message to later publish


**Return**: 

### function reset

```cpp
void reset()
```

Reset function. Sets the integral error term to 0. 

### function setFFGains

```cpp
void setFFGains(
    const float & ff_gain,
    const float & ff_d_gain,
    const float & ff_lin_drag_gain,
    const float & ff_quad_drag_gain
)
```

Set the feedfoward Gains object. 

**Parameters**: 

  * **ff_gain** Feefoward gain of reference 
  * **ff_d_gain** Feedoforward gain of reference derivative 
  * **ff_lin_drag_gain** Feefoward gain (for linear drag) 
  * **ff_quad_drag_gain** Feefoward gain (for quadratic drag) 


### function setGains

```cpp
void setGains(
    const float & kp,
    const float & ki,
    const float & kd
)
```

Set the Gains object. 

**Parameters**: 

  * **kp** Proportional gain 
  * **ki** Integral gain 
  * **kd** Derivative gain 


### function setLimitBounds

```cpp
void setLimitBounds(
    const float & max_out,
    const float & min_out
)
```

Set the Limit Bounds object. 

**Parameters**: 

  * **max_out** maximum output allowed 
  * **min_out** minimum output allowed 


### function getGains

```cpp
std::vector< double > getGains() const
```

Get the Gains object. 

**Return**: std::vector<float> const 

### function getLimitBounds

```cpp
std::vector< double > getLimitBounds() const
```

Get the Limit Bounds object. 

**Return**: std::vector<float> const 

### function getDebugInfo

```cpp
medusa_msgs::mPidDebug getDebugInfo() const
```

Get debug info from the PID controller internal variables. 

**Return**: std::vector<float> const 

## Public Attributes Documentation

### variable disable

```cpp
bool disable;
```


## Protected Attributes Documentation

### variable p_gain_

```cpp
float p_gain_;
```


### variable i_gain_

```cpp
float i_gain_;
```


### variable d_gain_

```cpp
float d_gain_;
```


### variable ff_gain_

```cpp
float ff_gain_;
```


### variable ff_d_gain_

```cpp
float ff_d_gain_;
```


### variable ff_lin_drag_gain_

```cpp
float ff_lin_drag_gain_;
```


### variable ff_quad_drag_gain_

```cpp
float ff_quad_drag_gain_;
```


### variable max_error_

```cpp
float max_error_;
```


### variable max_out_

```cpp
float max_out_;
```


### variable min_error_

```cpp
float min_error_;
```


### variable min_out_

```cpp
float min_out_;
```


### variable integral_

```cpp
float integral_;
```


### variable pre_error_

```cpp
float pre_error_;
```


### variable prev_ref_value_

```cpp
float prev_ref_value_;
```


### variable has_lpf_

```cpp
bool has_lpf_ {false};
```


### variable lpf_

```cpp
std::unique_ptr< LowPassFilter > lpf_;
```


### variable msg_debug_

```cpp
medusa_msgs::mPidDebug msg_debug_;
```


-------------------------------

Updated on 2022-05-30 at 08:04:28 +0000