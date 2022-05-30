---
title: diagnostic_aggregator::StatusAnalyzer
summary: Class StatusAnalyzer derived from AnalyzerGroup. 

---

# diagnostic_aggregator::StatusAnalyzer



Class [StatusAnalyzer]() derived from AnalyzerGroup. 


`#include <StatusAnalyzer.h>`

Inherits from AnalyzerGroup

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[StatusAnalyzer](/medusa_base/api/markdown/medusa_addons/status_aggregator/Classes/classdiagnostic__aggregator_1_1StatusAnalyzer/#function-statusanalyzer)**()<br>Construct a new Status Analyzer object.  |
| bool | **[init](/medusa_base/api/markdown/medusa_addons/status_aggregator/Classes/classdiagnostic__aggregator_1_1StatusAnalyzer/#function-init)**(const std::string base_path, const ros::NodeHandle & n) override<br>Initializes [StatusAnalyzer](/medusa_base/api/markdown/medusa_addons/status_aggregator/Classes/classdiagnostic__aggregator_1_1StatusAnalyzer/) from namespace.  |
| std::vector< diagnostic_msgs::DiagnosticStatusPtr > | **[report](/medusa_base/api/markdown/medusa_addons/status_aggregator/Classes/classdiagnostic__aggregator_1_1StatusAnalyzer/#function-report)**() override<br>Reports current state, returns vector of formatted status messages.  |

## Public Functions Documentation

### function StatusAnalyzer

```cpp
StatusAnalyzer()
```

Construct a new Status Analyzer object. 

### function init

```cpp
bool init(
    const std::string base_path,
    const ros::NodeHandle & n
) override
```

Initializes [StatusAnalyzer](/medusa_base/api/markdown/medusa_addons/status_aggregator/Classes/classdiagnostic__aggregator_1_1StatusAnalyzer/) from namespace. 

**Parameters**: 

  * **base_path** : Prefix for all analyzers (ex: 'Sensors') 
  * **n** : NodeHandle in full namespace 


**Return**: True if initialization succeed, false if no errors 

### function report

```cpp
std::vector< diagnostic_msgs::DiagnosticStatusPtr > report() override
```

Reports current state, returns vector of formatted status messages. 

**Return**: std::vector<diagnostic_msgs::DiagnosticStatusPtr> 

-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000