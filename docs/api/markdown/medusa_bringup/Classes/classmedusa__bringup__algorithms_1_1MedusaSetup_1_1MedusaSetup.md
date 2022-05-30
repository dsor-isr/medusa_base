---
title: medusa_bringup_algorithms::MedusaSetup::MedusaSetup

---

# medusa_bringup_algorithms::MedusaSetup::MedusaSetup





## Public Functions

|                | Name           |
| -------------- | -------------- |
| def | **[__init__](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function---init--)**(self self, vehicle_name vehicle_name, config_package_path config_package_path, folder folder, namespace namespace, vehicle_configuration vehicle_configuration =None) |
| def | **[publish_process_state](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-publish-process-state)**(self self, event event) |
| def | **[callback_manage_process](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-callback-manage-process)**(self self, req req) |
| def | **[clean_ros_processes](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-clean-ros-processes)**(self self) |
| def | **[create_processes](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-create-processes)**(self self) |
| def | **[start_init_processes](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-start-init-processes)**(self self) |
| def | **[start_all_processes](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-start-all-processes)**(self self) |
| def | **[stop_all_processes](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-stop-all-processes)**(self self) |
| def | **[get_process_from_name](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-get-process-from-name)**(self self, name name) |
| def | **[get_dependency_process_list](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-get-dependency-process-list)**(self self, process process) |
| def | **[are_dependencies_met](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-are-dependencies-met)**(self self, process process) |
| def | **[check_and_start_dependencies](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-check-and-start-dependencies)**(self self, process process) |
| def | **[start_process](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-start-process)**(self self, p p, start_dependencies start_dependencies =False) |
| def | **[start_process_from_name](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-start-process-from-name)**(self self, name name, start_dependencies start_dependencies =False) |
| def | **[stop_process_from_name](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-stop-process-from-name)**(self self, name name) |
| def | **[restart_process_from_name](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-restart-process-from-name)**(self self, name name) |
| def | **[kill_process_from_name](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-kill-process-from-name)**(self self, name name) |
| def | **[create_response](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-create-response)**(status status, message message) |
| def | **[stop_process](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-stop-process)**(p p) |
| def | **[restart_process](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-restart-process)**(p p) |
| def | **[kill_process](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#function-kill-process)**(p p) |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| | **[vehicle_name](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#variable-vehicle-name)**  |
| | **[config_package_path](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#variable-config-package-path)**  |
| | **[folder](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#variable-folder)**  |
| | **[namespace](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#variable-namespace)**  |
| | **[vehicle_configuration](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#variable-vehicle-configuration)**  |
| | **[process_list](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#variable-process-list)**  |
| | **[process_config](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#variable-process-config)**  |
| | **[process_state_publish_rate](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#variable-process-state-publish-rate)**  |
| | **[process_state_publisher](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#variable-process-state-publisher)**  |
| | **[manage_process_server](/medusa_base/api/markdown/medusa_bringup/Classes/classmedusa__bringup__algorithms_1_1MedusaSetup_1_1MedusaSetup/#variable-manage-process-server)**  |

## Public Functions Documentation

### function __init__

```python
def __init__(
    self self,
    vehicle_name vehicle_name,
    config_package_path config_package_path,
    folder folder,
    namespace namespace,
    vehicle_configuration vehicle_configuration =None
)
```


### function publish_process_state

```python
def publish_process_state(
    self self,
    event event
)
```


### function callback_manage_process

```python
def callback_manage_process(
    self self,
    req req
)
```


### function clean_ros_processes

```python
def clean_ros_processes(
    self self
)
```


### function create_processes

```python
def create_processes(
    self self
)
```


### function start_init_processes

```python
def start_init_processes(
    self self
)
```


### function start_all_processes

```python
def start_all_processes(
    self self
)
```


### function stop_all_processes

```python
def stop_all_processes(
    self self
)
```


### function get_process_from_name

```python
def get_process_from_name(
    self self,
    name name
)
```


### function get_dependency_process_list

```python
def get_dependency_process_list(
    self self,
    process process
)
```


### function are_dependencies_met

```python
def are_dependencies_met(
    self self,
    process process
)
```


### function check_and_start_dependencies

```python
def check_and_start_dependencies(
    self self,
    process process
)
```


### function start_process

```python
def start_process(
    self self,
    p p,
    start_dependencies start_dependencies =False
)
```


### function start_process_from_name

```python
def start_process_from_name(
    self self,
    name name,
    start_dependencies start_dependencies =False
)
```


### function stop_process_from_name

```python
def stop_process_from_name(
    self self,
    name name
)
```


### function restart_process_from_name

```python
def restart_process_from_name(
    self self,
    name name
)
```


### function kill_process_from_name

```python
def kill_process_from_name(
    self self,
    name name
)
```


### function create_response

```python
static def create_response(
    status status,
    message message
)
```


### function stop_process

```python
static def stop_process(
    p p
)
```


### function restart_process

```python
static def restart_process(
    p p
)
```


### function kill_process

```python
static def kill_process(
    p p
)
```


## Public Attributes Documentation

### variable vehicle_name

```python
vehicle_name;
```


### variable config_package_path

```python
config_package_path;
```


### variable folder

```python
folder;
```


### variable namespace

```python
namespace;
```


### variable vehicle_configuration

```python
vehicle_configuration;
```


### variable process_list

```python
process_list;
```


### variable process_config

```python
process_config;
```


### variable process_state_publish_rate

```python
process_state_publish_rate;
```


### variable process_state_publisher

```python
process_state_publisher;
```


### variable manage_process_server

```python
manage_process_server;
```


-------------------------------

Updated on 2022-05-30 at 08:04:26 +0000