---
title: MedusaDiagnostics
summary: MedusaDiagnostics namespace. 

---

# MedusaDiagnostics

[MedusaDiagnostics]() namespace. 

## Functions

|                | Name           |
| -------------- | -------------- |
| diagnostic_msgs::DiagnosticStatus | **[setDiagnosisMsg](/medusa_base/api/markdown/medusa_addons/libraries/medusa_diagnostics_library/Namespaces/namespaceMedusaDiagnostics/#function-setdiagnosismsg)**(const uint8_t & level, const std::string & name, const std::string & message, const std::string & hardware_id)<br>Set the Diagnosis Msg object (DiagnosticStatus)  |
| void | **[addKeyValue](/medusa_base/api/markdown/medusa_addons/libraries/medusa_diagnostics_library/Namespaces/namespaceMedusaDiagnostics/#function-addkeyvalue)**(diagnostic_msgs::DiagnosticArray * diagnostic_msg, const std::string & key_name, const std::string & value, const unsigned int & index)<br>add key values to diagnostic message  |
| template <typename T \> <br>bool | **[checkLowerBound](/medusa_base/api/markdown/medusa_addons/libraries/medusa_diagnostics_library/Namespaces/namespaceMedusaDiagnostics/#function-checklowerbound)**(const T & value, const T & lower_bound)<br>Check lower bound value.  |
| template <typename T \> <br>bool | **[checkUpperBound](/medusa_base/api/markdown/medusa_addons/libraries/medusa_diagnostics_library/Namespaces/namespaceMedusaDiagnostics/#function-checkupperbound)**(const T & value, const T & upper_bound) |
| void | **[warnLevel](/medusa_base/api/markdown/medusa_addons/libraries/medusa_diagnostics_library/Namespaces/namespaceMedusaDiagnostics/#function-warnlevel)**(diagnostic_msgs::DiagnosticArray * diagnostic_msg, const std::string & message, const unsigned int & index)<br>Define the level as WARN and change the message in diagnostics.  |
| void | **[errorLevel](/medusa_base/api/markdown/medusa_addons/libraries/medusa_diagnostics_library/Namespaces/namespaceMedusaDiagnostics/#function-errorlevel)**(diagnostic_msgs::DiagnosticArray * diagnostic_msg, const std::string & message, const unsigned int & index)<br>Define the level as Error and change the message in diagnostics.  |


## Functions Documentation

### function setDiagnosisMsg

```cpp
diagnostic_msgs::DiagnosticStatus setDiagnosisMsg(
    const uint8_t & level,
    const std::string & name,
    const std::string & message,
    const std::string & hardware_id
)
```

Set the Diagnosis Msg object (DiagnosticStatus) 

**Parameters**: 

  * **level** 0->OK, 1-> WARN, 2-> ERROR, 3->STALE 
  * **name** name of what sensor/node is being diagnosed, ex: /sensors/ + node_name 
  * **message** Say if it is ok or not 
  * **hardware_id** name of the sensor 


**Return**: diagnostic_msgs::DiagnosticStatus 

### function addKeyValue

```cpp
void addKeyValue(
    diagnostic_msgs::DiagnosticArray * diagnostic_msg,
    const std::string & key_name,
    const std::string & value,
    const unsigned int & index
)
```

add key values to diagnostic message 

**Parameters**: 

  * **diagnostic_msg** array(DiagnosticArray) of diagnostic_msgs 
  * **key_name** name of what we are diagnosing ex: Temperature, Current, Yaw 
  * **value** value being diagnosed, ex: from sensor 
  * **index** of the diagnostic_msg array 


### function checkLowerBound

```cpp
template <typename T >
bool checkLowerBound(
    const T & value,
    const T & lower_bound
)
```

Check lower bound value. 

**Parameters**: 

  * **value** value being diagnosed, ex: received by the sensor 
  * **lower_bound** value of lower bound defined by the user 


**Template Parameters**: 

  * **T** type of the values to be compared (int, double, float) 


**Return**: 

  * true if the sensor value is lower than the lower bound 
  * false if the sensor value is bigger than the lower bound 


### function checkUpperBound

```cpp
template <typename T >
bool checkUpperBound(
    const T & value,
    const T & upper_bound
)
```


**Parameters**: 

  * **value** received by the sensor 
  * **upper_bound** value of the upper bound defined by the user 


**Template Parameters**: 

  * **T** type of the values to be compared (int, double, float) 


**Return**: 

  * true if the sensor value is bigger than the upper bound 
  * false if the sensor value is lower thant the upper bound 


### function warnLevel

```cpp
void warnLevel(
    diagnostic_msgs::DiagnosticArray * diagnostic_msg,
    const std::string & message,
    const unsigned int & index
)
```

Define the level as WARN and change the message in diagnostics. 

**Parameters**: 

  * **diagnostic_msg** array(DiagnosticArray) of diagnostic_msgs 
  * **message** to clarify warning 
  * **index** of the diagnostic_msg array 


### function errorLevel

```cpp
void errorLevel(
    diagnostic_msgs::DiagnosticArray * diagnostic_msg,
    const std::string & message,
    const unsigned int & index
)
```

Define the level as Error and change the message in diagnostics. 

**Parameters**: 

  * **diagnostic_msg** array(DiagnosticArray) of diagnostic_msgs 
  * **message** to clarify error 
  * **index** of the diagnostic_msg array 






-------------------------------

Updated on 2022-05-30 at 08:04:26 +0000