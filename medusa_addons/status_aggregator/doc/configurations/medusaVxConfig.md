# medusa_vx config - status_aggregator

## Run
This package is launched with medusa stack.
```
ex:
roslaunch medusa_bringup medusa_bringup.launch name:= mvector docks:=missions
```

Please check if **aggregator** is set to true in the vehicle configurations. For this, please check the following block in *medusa_bringup/config/docks/mvector/process.yaml* file:

```yaml
 - name: 'addons'
        launch_on_startup: true
        delay_before_start: 0.0
        cmd: 'roslaunch medusa_bringup addons.launch'
        args:
              - **aggregator:= true**
              - console_path_parser:= true
              - console_server:= true
              - wifi_acoustic_safety:= true
              - rosbridge_server:= false
```


## Launch file
You have a main launch (**medusa_bringup.launch**) that calls a node (**medusa_bringup_node**) responsible to manage all the processes in a **proecss.yaml** file.
The node while running is responsible to launch a subset of other launch files located at */medusa_bringup/launch/dev_launch*. Among the exisitng ones, here the focus is in the **addons.launch** where **aggregator** is present.

Looking in some detail to the file we point out the key points:
```xml
    <arg name="name"                />   
    <arg name="mission"             default="example"   />   
    <arg name="wifi_acoustic_safety"     default="false"     />   <!-- safety features -->
    **<arg name="console_path_parser"         default="false"     />**   
    <arg name="http_server"         default="false"     />   
    <arg name="aggregator"          default="false"     />   
    <arg name="rosbridge_server"    default="false"     />   
    <arg name="websocket_port"      default="9090"      />       <!-- arg to use in http-->
```

By default all the nodes in the **medusa_addons** package are set to false and won't run if you don't change the corresponding boolean in the **process.yaml** like in the previous section. 

```
 <group ns="addons">
```
Every addon is part of the group **addons**, which means that topics will be someting like */addons/node/some_topic*, here */addons/aggregator/some_topic*.

```xml
<group if="$(arg aggregator)">
            <node pkg="diagnostic_aggregator" type="aggregator_node" name="aggregator" output="screen" />
</group>
```

As you can see the *if* guarantees that the node only starts if in the **process.yaml**, **aggregator** is set to true.  


## Configurations

Configuration is divided between two files. The first one, *medusa_bringup/config/dev_configs/ros.yaml*, is responsible with ros node items (frequency and topics names). Below, the section of **console_path_parser**:


```yaml
addons/aggregator:
    pub_rate: 2.0
```

The second file, */medusa_bringup/config/mission/vehicle/addons.yaml*, accommodates items more related with the node inner-workings. 
Below, the section of **aggregator**:
```yaml
*file, ex: /medusa_bringup/config/docks/mvector/addons.yaml

aggregator:
    analyzers:
        sensors:
            type: diagnostic_aggregator/StatusAnalyzer
            path: Sensors
            analyzers:
                leaks:    
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: Leaks
                    contains: Leaks
                pressure:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: Pressure
                    contains: Pressure
                temp:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: Temperature
                    contains: Temperature
                imu:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: IMU
                    contains: IMU
                gps:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: GPS
                    contains: GPS
                altimeter:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: Altimeter
                    contains: Altimeter
                depth_cell:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: DepthCell
                    contains: DepthCell
        actuators:
            type: diagnostic_aggregator/StatusAnalyzer
            path: Actuators
            analyzers:
                thruster_0:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: Thruster0
                    contains: Thruster0
                thruster_1:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: Thruster1
                    contains: Thruster1
                thruster_2:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: Thruster2
                    contains: Thruster2
                thruster_3:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: Thruster3
                    contains: Thruster3
                thruster_4:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: Thruster4
                    contains: Thruster4
                thruster_5:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: Thruster5
                    contains: Thruster5
        power_system:
            type: diagnostic_aggregator/StatusAnalyzer
            path: Power System
            analyzers:
                batmonit:
                    type: diagnostic_aggregator/GenericAnalyzer
                    path: Batmonit
                    contains: Batmonit
```  