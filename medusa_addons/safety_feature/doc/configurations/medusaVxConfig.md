# medusa_vx config - wifi_acoustic_safety 

## Run

### Vehicle Side
This package is launched with medusa stack
```
ex:
roslaunch medusa_bringup medusa_bringup.launch name:= mvector docks:=missions
```

Please check if **wifi_acoustic_safety** is set to true in the vehicle configurations. For this, please check the following block in *medusa_bringup/config/docks/mvector/process.yaml* file:

```yaml
 - name: 'addons'
        launch_on_startup: true
        delay_before_start: 0.0
        cmd: 'roslaunch medusa_bringup addons.launch'
        args:
              - aggregator:= true
              - console_path_parser:= true
              - console_server:= true
              - **wifi_acoustic_safety:= true**
              - rosbridge_server:= false
```


## Launch file
You have a main launch (**medusa_bringup.launch**) that calls a node (**medusa_bringup_node**) responsible to manage all the processes in a **proecss.yaml** file.
The node while running is responsible to launch a subset of other launch files located at */medusa_bringup/launch/dev_launch*. Among the exisitng ones, here the focus is in the **addons.launch** where **wifi_acoustic_safety** is present.

Looking in some detail to the file we point out the key points:
```xml
    <arg name="name"                />   
    <arg name="mission"             default="example"   />   
    **<arg name="wifi_acoustic_safety"     default="false"     />**   <!-- safety features -->
    <arg name="console_path_parser"         default="false"     />   
    <arg name="http_server"         default="false"     />   
    <arg name="aggregator"          default="false"     />   
    <arg name="rosbridge_server"    default="false"     />   
    <arg name="websocket_port"      default="9090"      />       <!-- arg to use in http-->
```

By default all the nodes in the **medusa_addons** package are set to false and won't run if you don't change the corresponding boolean in the **process.yaml** like in the previous section. 

```
 <group ns="addons">
```
Every addon is part of the group **addons**, which means that almost all topics will be someting like */addons/node/some_topic*, here */addons/wifi_acoustic_safety/some_topic*.

```xml
<!-- ############################ -->
<!-- Acoustic Safety -->
<!-- ############################ -->
<group if="$(arg wifi_acoustic_safety)">
    <node pkg="safety_feature" type="wifi_acoustic_safety" name="wifi_acoustic_safety" output="screen">
	    <param name="name"              value="$(arg name)" />
		<param name="surface_vehicle"   value="dummy"/>
    </node>
</group>p
```

As you can see the *if* guarantees that the node only starts if in the **process.yaml**, **wifi_acoustic_safety** is set to true.  


## Configurations

Configuration is divided between two files. The first one, *medusa_bringup/config/mission/vehicle.yaml*.Below, the section of **wifi_acoustic_safety**, for the **docks** mission and **mvector** vehicle:


```yaml
wifi_acoustic_safety:
    SAFETY_PORT: 10111
    RATE: 1.0
```

The second file, */medusa_bringup/config/mission/vehicle/common.yaml*, accommodates the delay times for wifi and acoustic. 
Below, the important parameter for of **wifi_acoustic_safety**:
```yaml
*file, ex: /medusa_bringup/config/docks/mvector/common.yaml
  safety_features:
    max_depth: 5.0 # [m]
    min_altitude: 1.0 # [m]
    min_altitude_ref: 1.5 # [m]
    **wifi_timeout: 10.0 # [s]**
    **acoustic_timeout: 15.0 # [s]**
```

### Client Side
In a console pc please run:
```
rosrun safety_feature SafetyFeatureCLT ip_vehicle
or 
catkin_ws/src/medusa_vx/medusa_addons/safety_feature/src/Client/SafetyFeatureCLT ip_vehicle

ex: mvector
rosrun safety_feature SafetyFeatureCLT 192.168.1.33
```

This will guarantee that the safety is disable(0), as soon you losse connection with vehicle it will enable safety(1) and thrusters will stop.

