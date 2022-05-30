---
title: MedusaGimmicks
summary: MedusaGimmicks namespace. 

---

# MedusaGimmicks

[MedusaGimmicks]() namespace.  [More...](#detailed-description)

## Functions

|                | Name           |
| -------------- | -------------- |
| template <typename T \> <br>T | **[getParameters](/medusa_base/api/markdown/medusa_addons/libraries/medusa_gimmicks_library/Namespaces/namespaceMedusaGimmicks/#function-getparameters)**(ros::NodeHandle & _nh, std::string const & parameter_name)<br>Get the Parameters object.  |
| template <typename T \> <br>T | **[getParameters](/medusa_base/api/markdown/medusa_addons/libraries/medusa_gimmicks_library/Namespaces/namespaceMedusaGimmicks/#function-getparameters)**(ros::NodeHandle & _nh, std::string const & parameter_name, T default_value)<br>Get the Parameters object.  |
| template <typename T \> <br>T | **[getParameters](/medusa_base/api/markdown/medusa_addons/libraries/medusa_gimmicks_library/Namespaces/namespaceMedusaGimmicks/#function-getparameters)**(ros::NodeHandle & _nh, std::string const & parameter_name, T default_value, bool delete_param)<br>Get the Parameters object.  |
| void | **[spherical_to_cartesian](/medusa_base/api/markdown/medusa_addons/libraries/medusa_gimmicks_library/Namespaces/namespaceMedusaGimmicks/#function-spherical-to-cartesian)**(double bearing, double elevation, double range, double * out_pos_cart)<br>Convert from spherical to cartesian coordinates. Used mainly with usbl fixes.  |
| int | **[signVal](/medusa_base/api/markdown/medusa_addons/libraries/medusa_gimmicks_library/Namespaces/namespaceMedusaGimmicks/#function-signval)**(double v)<br>Returns the sign of a double.  |
| double | **[wrap2pi](/medusa_base/api/markdown/medusa_addons/libraries/medusa_gimmicks_library/Namespaces/namespaceMedusaGimmicks/#function-wrap2pi)**(double theta, const int mode)<br>Wraps angle between [0, 2PI] or [-PI, PI].  |
| double | **[wrapTo2pi](/medusa_base/api/markdown/medusa_addons/libraries/medusa_gimmicks_library/Namespaces/namespaceMedusaGimmicks/#function-wrapto2pi)**(double in)<br>Wrap angle between [0, 2PI].  |
| double | **[angleDiff](/medusa_base/api/markdown/medusa_addons/libraries/medusa_gimmicks_library/Namespaces/namespaceMedusaGimmicks/#function-anglediff)**(double a, double b)<br>Method to calculate the diference between angles correctly even if they wrap between -pi and pi.  |
| template <typename A ,typename B \> <br>void | **[publishValue](/medusa_base/api/markdown/medusa_addons/libraries/medusa_gimmicks_library/Namespaces/namespaceMedusaGimmicks/#function-publishvalue)**(ros::Publisher & pub, B & value) |

## Attributes

|                | Name           |
| -------------- | -------------- |
| const double | **[PI](/medusa_base/api/markdown/medusa_addons/libraries/medusa_gimmicks_library/Namespaces/namespaceMedusaGimmicks/#variable-pi)** <br>PI value.  |

## Detailed Description

[MedusaGimmicks]() namespace. 

**Note**: why the code of templates is here -> because linkage problems see [https://stackoverflow.com/a/1353981](https://stackoverflow.com/a/1353981)

## Functions Documentation

### function getParameters

```cpp
template <typename T >
T getParameters(
    ros::NodeHandle & _nh,
    std::string const & parameter_name
)
```

Get the Parameters object. 

**Parameters**: 

  * **_nh** ros nodehandle 
  * **parameter_name** string with paramenter name 


**Template Parameters**: 

  * **T** the type of data of a desired parameter 


**Return**: T parameter value

**Note**: Option not considering default value, so the config file must have the parameter; 

### function getParameters

```cpp
template <typename T >
T getParameters(
    ros::NodeHandle & _nh,
    std::string const & parameter_name,
    T default_value
)
```

Get the Parameters object. 

**Parameters**: 

  * **_nh** ros nodehandle 
  * **parameter_name** string with parameter name 
  * **default_value** default value of the parameter 


**Template Parameters**: 

  * **T** the type of data of a desired parameter 


**Return**: T parameter value

**Note**: Option considering default value. Even if the parameter doesn't exist in config file it is possible to use a default value. 

### function getParameters

```cpp
template <typename T >
T getParameters(
    ros::NodeHandle & _nh,
    std::string const & parameter_name,
    T default_value,
    bool delete_param
)
```

Get the Parameters object. 

**Parameters**: 

  * **_nh** ros nodehandle 
  * **parameter_name** string with parameter name 
  * **default_value** default value of the parameter 
  * **delete_param** boolean to delete or not the parameter from parameter server 


**Template Parameters**: 

  * **T** the type of data of a desired parameter 


**Return**: T parameter value

**Note**: Option considering default value. Even if the parameter doesn't exist in config file it is possible to use a default value. Removes parameter from parameter server 

### function spherical_to_cartesian

```cpp
void spherical_to_cartesian(
    double bearing,
    double elevation,
    double range,
    double * out_pos_cart
)
```

Convert from spherical to cartesian coordinates. Used mainly with usbl fixes. 

**Parameters**: 

  * **bearing** horizontal angle between the direction of an object and another object or between it and the true north direction in degrees. 
  * **elevation** angle measured between the horizontal and the vehicle line of sight to the object 
  * **range** distance to the object 
  * **out_pos_cart** cartesian coordinates pointer 


### function signVal

```cpp
int signVal(
    double v
)
```

Returns the sign of a double. 

**Parameters**: 

  * **v** double value 


**Return**: 

  * int 1 if value is positive 
  * int 0 if value is 0 
  * int -1 if value is negative 


### function wrap2pi

```cpp
double wrap2pi(
    double theta,
    const int mode
)
```

Wraps angle between [0, 2PI] or [-PI, PI]. 

**Parameters**: 

  * **theta** angle in radians 
  * **mode** 0 = Wrap from [0, 2*pi]; 1 = Wrap from [-pi, pi] 


**Return**: double wraped angle 

### function wrapTo2pi

```cpp
double wrapTo2pi(
    double in
)
```

Wrap angle between [0, 2PI]. 

**Parameters**: 

  * **in** angle in radians 


**Return**: double wraped angle 

### function angleDiff

```cpp
double angleDiff(
    double a,
    double b
)
```

Method to calculate the diference between angles correctly even if they wrap between -pi and pi. 

**Parameters**: 

  * **a** angle 1 in radians 
  * **b** angle 2 in radians 


**Return**: double 

### function publishValue

```cpp
template <typename A ,
typename B >
void publishValue(
    ros::Publisher & pub,
    B & value
)
```


**Parameters**: 

  * **pub** publisher 
  * **value** value 


**Template Parameters**: 

  * **A** value type 
  * **B** publisher type 



## Attributes Documentation

### variable PI

```cpp
const double PI = 3.14159265;
```

PI value. 




-------------------------------

Updated on 2022-05-30 at 08:04:27 +0000