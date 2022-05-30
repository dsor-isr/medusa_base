---
title: ConsolePathParserNode
summary: Class Responsible for parsing a mission from the console Yebisu to medusa_vx stack format. 

---

# ConsolePathParserNode



Class Responsible for parsing a mission from the console Yebisu to medusa_vx stack format. 


`#include <ConsolePathParserNode.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[ConsolePathParserNode](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#function-consolepathparsernode)**(ros::NodeHandle * nodehandle, ros::NodeHandle * nodehandle_private)<br>Construct a new Console Path Parser Node object.  |
| | **[~ConsolePathParserNode](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#function-~consolepathparsernode)**()<br>Destroy the Console Path Parser Node object.  |
| double | **[nodeFrequency](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#function-nodefrequency)**()<br>Method to setup the frequency of the node.  |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| std::list< [Section](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/) > | **[mission](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-mission)**  |
| std::list< [Formation](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classFormation/) > | **[formation](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-formation)**  |
| std::list< [Section](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classSection/) >::iterator | **[act_section](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-act-section)**  |
| medusa_msgs::Section | **[section_copy](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-section-copy)**  |
| ros::Time | **[depth_end](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-depth-end)**  |
| int | **[own_id](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-own-id)**  |
| double | **[xrefpoint](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-xrefpoint)**  |
| double | **[yrefpoint](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-yrefpoint)**  |
| double | **[gamma_s](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-gamma-s)**  |
| double | **[gamma_e](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-gamma-e)**  |
| double | **[x_act](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-x-act)**  |
| double | **[y_act](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-y-act)**  |
| double | **[gamma](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-gamma)**  |
| double | **[gamma_old](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-gamma-old)**  |
| double | **[u_est](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-u-est)**  |
| double | **[x_forma](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-x-forma)**  |
| double | **[y_forma](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-y-forma)**  |
| float | **[DesiredDepth](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-desireddepth)**  |
| bool | **[wpOrient](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-wporient)**  |
| bool | **[ENABLE](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-enable)**  |
| bool | **[formation_mode](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-formation-mode)**  |
| bool | **[biased_formation_mode](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-biased-formation-mode)**  |
| float | **[node_frequency](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-node-frequency)**  |
| std::string | **[path_folder](/medusa_base/api/markdown/medusa_addons/console_path_parser/Classes/classConsolePathParserNode/#variable-path-folder)**  |

## Public Functions Documentation

### function ConsolePathParserNode

```cpp
ConsolePathParserNode(
    ros::NodeHandle * nodehandle,
    ros::NodeHandle * nodehandle_private
)
```

Construct a new Console Path Parser Node object. 

**Parameters**: 

  * **nodehandle** 
  * **nodehandle_private** 
  * **nodehandle** the public ros nodehandle 
  * **nodehandle_private** the private ros nodehandle 


Console Path Parser node constructor.


### function ~ConsolePathParserNode

```cpp
~ConsolePathParserNode()
```

Destroy the Console Path Parser Node object. 

Console Path Parser node destructor.


### function nodeFrequency

```cpp
double nodeFrequency()
```

Method to setup the frequency of the node. 

**Return**: double 

## Public Attributes Documentation

### variable mission

```cpp
std::list< Section > mission;
```


### variable formation

```cpp
std::list< Formation > formation;
```


### variable act_section

```cpp
std::list< Section >::iterator act_section;
```


### variable section_copy

```cpp
medusa_msgs::Section section_copy;
```


### variable depth_end

```cpp
ros::Time depth_end;
```


### variable own_id

```cpp
int own_id {0};
```


### variable xrefpoint

```cpp
double xrefpoint = 0;
```


### variable yrefpoint

```cpp
double yrefpoint = 0;
```


### variable gamma_s

```cpp
double gamma_s = 0;
```


### variable gamma_e

```cpp
double gamma_e = 0;
```


### variable x_act

```cpp
double x_act = 0;
```


### variable y_act

```cpp
double y_act = 0;
```


### variable gamma

```cpp
double gamma = 0;
```


### variable gamma_old

```cpp
double gamma_old = 0;
```


### variable u_est

```cpp
double u_est = 0;
```


### variable x_forma

```cpp
double x_forma = 0;
```


### variable y_forma

```cpp
double y_forma = 0;
```


### variable DesiredDepth

```cpp
float DesiredDepth = 0.0;
```


### variable wpOrient

```cpp
bool wpOrient;
```


### variable ENABLE

```cpp
bool ENABLE = false;
```


### variable formation_mode

```cpp
bool formation_mode = false;
```


### variable biased_formation_mode

```cpp
bool biased_formation_mode = false;
```


### variable node_frequency

```cpp
float node_frequency;
```


### variable path_folder

```cpp
std::string path_folder;
```


-------------------------------

Updated on 2022-05-30 at 08:04:30 +0000