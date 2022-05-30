---
title: RemoteControllerNode::RemoteControllerNode

---

# RemoteControllerNode::RemoteControllerNode



 [More...](#detailed-description)

## Public Functions

|                | Name           |
| -------------- | -------------- |
| def | **[__init__](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#function---init--)**(self self) |
| def | **[timerCallback](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#function-timercallback)**(self self, event event) |
| def | **[initializeTimer](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#function-initializetimer)**(self self) |
| def | **[initializeSubscribers](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#function-initializesubscribers)**(self self) |
| def | **[initializePublishers](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#function-initializepublishers)**(self self) |
| def | **[initializeJoystick](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#function-initializejoystick)**(self self) |
| def | **[state_callback](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#function-state-callback)**(self self, msg msg) |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| | **[node_frequency](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#variable-node-frequency)**  |
| | **[h_timerActivate](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#variable-h-timeractivate)**  |
| | **[yaw_state_](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#variable-yaw-state-)**  |
| | **[timer](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#variable-timer)**  |
| | **[state_sub](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#variable-state-sub)**  |
| | **[surge_pub](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#variable-surge-pub)**  |
| | **[sway_pub](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#variable-sway-pub)**  |
| | **[heave_pub](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#variable-heave-pub)**  |
| | **[yaw_rate_pub](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#variable-yaw-rate-pub)**  |
| | **[yaw_pub](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#variable-yaw-pub)**  |
| | **[depth_pub](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#variable-depth-pub)**  |
| | **[control_assignment](/medusa_base/api/markdown/medusa_addons/remote_controller/Classes/classRemoteControllerNode_1_1RemoteControllerNode/#variable-control-assignment)**  |

## Detailed Description

```python
class RemoteControllerNode::RemoteControllerNode;
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


### function initializeTimer

```python
def initializeTimer(
    self self
)
```




```
Method that starts the system timer that periodically calls a callback
```


### function initializeSubscribers

```python
def initializeSubscribers(
    self self
)
```




```
Method that initializes the ROS subscribers (to receive the current state of the AUV)
```


### function initializePublishers

```python
def initializePublishers(
    self self
)
```




```
Method that initializes the ROS publishers (to publish the references for the inner-loops)
```


### function initializeJoystick

```python
def initializeJoystick(
    self self
)
```




```
Method that initializes the joystick driver (using pygame)
```


### function state_callback

```python
def state_callback(
    self self,
    msg msg
)
```




```
Callback that is called when a message with the current state of the AUV is received.
Currently only the yaw orientation is saved (used to switch between yaw and yaw-rate controllers)
:param msg: NavigationStatus message
```


## Public Attributes Documentation

### variable node_frequency

```python
node_frequency;
```


### variable h_timerActivate

```python
h_timerActivate;
```


### variable yaw_state_

```python
yaw_state_;
```


### variable timer

```python
timer;
```


### variable state_sub

```python
state_sub;
```


### variable surge_pub

```python
surge_pub;
```


### variable sway_pub

```python
sway_pub;
```


### variable heave_pub

```python
heave_pub;
```


### variable yaw_rate_pub

```python
yaw_rate_pub;
```


### variable yaw_pub

```python
yaw_pub;
```


### variable depth_pub

```python
depth_pub;
```


### variable control_assignment

```python
control_assignment;
```


-------------------------------

Updated on 2022-05-30 at 08:04:28 +0000