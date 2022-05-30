---
title: AcousticPinger

---

# AcousticPinger





## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[AcousticPinger](/medusa_base/api/markdown/medusa_comms/comms_acoustic/interrogation_scheme/Classes/classAcousticPinger/#function-acousticpinger)**(ros::NodeHandle * nodehandle, ros::NodeHandle * nodehandle_private) |
| | **[~AcousticPinger](/medusa_base/api/markdown/medusa_comms/comms_acoustic/interrogation_scheme/Classes/classAcousticPinger/#function-~acousticpinger)**() |
| void | **[updateClock](/medusa_base/api/markdown/medusa_comms/comms_acoustic/interrogation_scheme/Classes/classAcousticPinger/#function-updateclock)**(unsigned long int tmodem, ros::Time tros) |
| unsigned long int | **[getModemClockNow](/medusa_base/api/markdown/medusa_comms/comms_acoustic/interrogation_scheme/Classes/classAcousticPinger/#function-getmodemclocknow)**() |
| void | **[triggerSerialization](/medusa_base/api/markdown/medusa_comms/comms_acoustic/interrogation_scheme/Classes/classAcousticPinger/#function-triggerserialization)**() |
| void | **[pingNextNode](/medusa_base/api/markdown/medusa_comms/comms_acoustic/interrogation_scheme/Classes/classAcousticPinger/#function-pingnextnode)**() |
| void | **[serializerCallback](/medusa_base/api/markdown/medusa_comms/comms_acoustic/interrogation_scheme/Classes/classAcousticPinger/#function-serializercallback)**(const std_msgs::String & msg) |
| void | **[EnableCallback](/medusa_base/api/markdown/medusa_comms/comms_acoustic/interrogation_scheme/Classes/classAcousticPinger/#function-enablecallback)**(const std_msgs::Bool & msg) |
| void | **[RECVIMSCallback](/medusa_base/api/markdown/medusa_comms/comms_acoustic/interrogation_scheme/Classes/classAcousticPinger/#function-recvimscallback)**(const dmac::DMACPayload & msg) |
| void | **[Timer](/medusa_base/api/markdown/medusa_comms/comms_acoustic/interrogation_scheme/Classes/classAcousticPinger/#function-timer)**(const ros::TimerEvent & e) |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| ros::NodeHandle | **[nh](/medusa_base/api/markdown/medusa_comms/comms_acoustic/interrogation_scheme/Classes/classAcousticPinger/#variable-nh)**  |

## Public Functions Documentation

### function AcousticPinger

```cpp
AcousticPinger(
    ros::NodeHandle * nodehandle,
    ros::NodeHandle * nodehandle_private
)
```


### function ~AcousticPinger

```cpp
~AcousticPinger()
```


### function updateClock

```cpp
void updateClock(
    unsigned long int tmodem,
    ros::Time tros
)
```


### function getModemClockNow

```cpp
unsigned long int getModemClockNow()
```


### function triggerSerialization

```cpp
void triggerSerialization()
```


### function pingNextNode

```cpp
void pingNextNode()
```


### function serializerCallback

```cpp
void serializerCallback(
    const std_msgs::String & msg
)
```


### function EnableCallback

```cpp
void EnableCallback(
    const std_msgs::Bool & msg
)
```


### function RECVIMSCallback

```cpp
void RECVIMSCallback(
    const dmac::DMACPayload & msg
)
```


### function Timer

```cpp
void Timer(
    const ros::TimerEvent & e
)
```


## Public Attributes Documentation

### variable nh

```cpp
ros::NodeHandle nh;
```


-------------------------------

Updated on 2022-05-30 at 08:04:28 +0000