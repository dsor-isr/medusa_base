---
title: pos2simmodem::Pos2SimModem

---

# pos2simmodem::Pos2SimModem



 [More...](#detailed-description)

Inherits from object

## Public Functions

|                | Name           |
| -------------- | -------------- |
| def | **[__init__](/medusa_base/api/markdown/medusa_comms/comms_acoustic/evo_tools/Classes/classpos2simmodem_1_1Pos2SimModem/#function---init--)**(self self) |
| def | **[position_callback](/medusa_base/api/markdown/medusa_comms/comms_acoustic/evo_tools/Classes/classpos2simmodem_1_1Pos2SimModem/#function-position-callback)**(self self, msg msg) |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| | **[address](/medusa_base/api/markdown/medusa_comms/comms_acoustic/evo_tools/Classes/classpos2simmodem_1_1Pos2SimModem/#variable-address)**  |
| | **[sock](/medusa_base/api/markdown/medusa_comms/comms_acoustic/evo_tools/Classes/classpos2simmodem_1_1Pos2SimModem/#variable-sock)**  |
| | **[time](/medusa_base/api/markdown/medusa_comms/comms_acoustic/evo_tools/Classes/classpos2simmodem_1_1Pos2SimModem/#variable-time)**  |
| | **[sub_odom](/medusa_base/api/markdown/medusa_comms/comms_acoustic/evo_tools/Classes/classpos2simmodem_1_1Pos2SimModem/#variable-sub-odom)**  |

## Detailed Description

```python
class pos2simmodem::Pos2SimModem;
```




```
Class to hold the read of Odometry messages and send position to the
simulated modem.
```

## Public Functions Documentation

### function __init__

```python
def __init__(
    self self
)
```




```
Initializing the necessary variables.
```


### function position_callback

```python
def position_callback(
    self self,
    msg msg
)
```




```
Reads Odometry and sends position.
```


## Public Attributes Documentation

### variable address

```python
address;
```


### variable sock

```python
sock;
```


### variable time

```python
time;
```


### variable sub_odom

```python
sub_odom;
```


-------------------------------

Updated on 2022-05-30 at 08:04:28 +0000