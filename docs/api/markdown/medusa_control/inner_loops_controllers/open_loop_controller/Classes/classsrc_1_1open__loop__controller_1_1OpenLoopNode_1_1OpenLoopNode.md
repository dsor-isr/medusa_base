---
title: src::open_loop_controller::OpenLoopNode::OpenLoopNode

---

# src::open_loop_controller::OpenLoopNode::OpenLoopNode



 [More...](#detailed-description)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| def | **[__init__](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#function---init--)**(self self) |
| def | **[load_params](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#function-load-params)**(self self) |
| def | **[initializeSubscribers](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#function-initializesubscribers)**(self self) |
| def | **[initializePublishers](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#function-initializepublishers)**(self self) |
| def | **[timerCallback](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#function-timercallback)**(self self, event event) |
| def | **[surgeCallback](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#function-surgecallback)**(self self, Float64 msg) |
| def | **[swayCallback](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#function-swaycallback)**(self self, Float64 msg) |
| def | **[heaveCallback](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#function-heavecallback)**(self self, Float64 msg) |
| def | **[yawRateCallback](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#function-yawratecallback)**(self self, Float64 msg) |
| def | **[initializeTimer](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#function-initializetimer)**(self self) |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| | **[h_timerActivate](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-h-timeractivate)**  |
| | **[surge_desired](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-surge-desired)**  |
| | **[sway_desired](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-sway-desired)**  |
| | **[heave_desired](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-heave-desired)**  |
| | **[yaw_rate_desired](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-yaw-rate-desired)**  |
| | **[last_update](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-last-update)**  |
| | **[node_frequency](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-node-frequency)**  |
| | **[gain_Fx](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-gain-fx)**  |
| | **[gain_Fy](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-gain-fy)**  |
| | **[gain_Fz](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-gain-fz)**  |
| | **[gain_Tz](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-gain-tz)**  |
| | **[surge_sub](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-surge-sub)**  |
| | **[sway_sub](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-sway-sub)**  |
| | **[heave_sub](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-heave-sub)**  |
| | **[yaw_rate_sub](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-yaw-rate-sub)**  |
| | **[force_pub](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-force-pub)**  |
| | **[timer](/medusa_base/api/markdown/medusa_control/inner_loops_controllers/open_loop_controller/Classes/classsrc_1_1open__loop__controller_1_1OpenLoopNode_1_1OpenLoopNode/#variable-timer)**  |

## Detailed Description

```python
class src::open_loop_controller::OpenLoopNode::OpenLoopNode;
```




```
Remote controller ROS node class. Receives from a joystick (using pygame) the desired controls for
the inner-loops and publishes these periodically (at a pre-defined frequency) to the inner-loops of the
vehicle
```

## Public Functions Documentation

### function __init__

```python
def __init__(
    self self
)
```




```
Class constructor. Initializes the ros node, loads the parameters from the ros parameter server, creates the inner-loops
publishers and initializes the timer that publishes the desired inputs to the inner-loops
```


### function load_params

```python
def load_params(
    self self
)
```


### function initializeSubscribers

```python
def initializeSubscribers(
    self self
)
```


### function initializePublishers

```python
def initializePublishers(
    self self
)
```


### function timerCallback

```python
def timerCallback(
    self self,
    event event
)
```




```
Callback used to publish to the inner-loops the desired control inputs
:param event: A timer event - unused but required by the ROS API
```


### function surgeCallback

```python
def surgeCallback(
    self self,
    Float64 msg
)
```


### function swayCallback

```python
def swayCallback(
    self self,
    Float64 msg
)
```


### function heaveCallback

```python
def heaveCallback(
    self self,
    Float64 msg
)
```


### function yawRateCallback

```python
def yawRateCallback(
    self self,
    Float64 msg
)
```


### function initializeTimer

```python
def initializeTimer(
    self self
)
```




```
Method that starts the system timer that periodically calls a callback
```


## Public Attributes Documentation

### variable h_timerActivate

```python
h_timerActivate;
```


### variable surge_desired

```python
surge_desired;
```


### variable sway_desired

```python
sway_desired;
```


### variable heave_desired

```python
heave_desired;
```


### variable yaw_rate_desired

```python
yaw_rate_desired;
```


### variable last_update

```python
last_update;
```


### variable node_frequency

```python
node_frequency;
```


### variable gain_Fx

```python
gain_Fx;
```


### variable gain_Fy

```python
gain_Fy;
```


### variable gain_Fz

```python
gain_Fz;
```


### variable gain_Tz

```python
gain_Tz;
```


### variable surge_sub

```python
surge_sub;
```


### variable sway_sub

```python
sway_sub;
```


### variable heave_sub

```python
heave_sub;
```


### variable yaw_rate_sub

```python
yaw_rate_sub;
```


### variable force_pub

```python
force_pub;
```


### variable timer

```python
timer;
```


-------------------------------

Updated on 2022-05-30 at 08:04:28 +0000