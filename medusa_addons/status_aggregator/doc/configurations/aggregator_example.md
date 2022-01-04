# Aggregator Example

For the aggregator present some hierarchy, you have to specify a yaml file with the analyzers.

*/status_aggregator/config/mvector*
```yaml
pub_rate: 2.0
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
        dvl:
            type: diagnostic_aggregator/GenericAnalyzer
            path: DVL
            contains: DVL
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
        thruster_3:
            type: diagnostic_aggregator/GenericAnalyzer
            path: Thruster3
            contains: Thruster3
        thruster_2:
            type: diagnostic_aggregator/GenericAnalyzer
            path: Thruster2
            contains: Thruster2
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
This yaml file can have 3 parameters:

* The pub_rate which is the frequency at which the aggregator publishes to the /diagnostics_agg topic.
* The base_path (and a secret one, path) which adds an additional root level in the hierarchy (e.g. /MyRobot/Sensors/IMU/…). But it’s useless, since there can only be one aggregator in the system, and so the separation between different robots cannot happen at this level. Thus, you can simply ignore it!
* The analyzers which is a list of… you guessed it, analyzers. The analyzers have a type, a name (a.k.a. path), and several options for matching criteria. The type can be **GenericAnalyzer** which does a grouping of status items based on their name or **StatusAnalyzer** which does a categorization of analyzers (e.g. Sensors/..., Actuators/..., Power System/...).

Now, comes the interesting part. What if we want to react to an error? We’ll have to write an analyzer that does this. But we won’t just write an analyzer. Since the categorization offered by the default analyzers is always relevant, we’ll extend the generic or group analyzers so that we maintain their functionality. The analyzers have two methods that can be of interest:

* The analyze method which can be useful for processing the data of a status message before we store it.
* The report method which can be used to make a decision based on the status messages to be reported by the analyzer.

In the accompanying code, there are two analyzers. One is based on the **GroupAnalyzer** and the other is based on the **GenericAnalyzer**. They both rely on the report method. They look at the reported statuses by their parent class and they publish a message if a condition is met. The **StatusAnalyzer** finds a group (/Sensors, /Actuator, or Power System) status and reacts when its level transitions from OK to something else. The **GenericAnalyzer** is the default one, but you can have specific ones for your needs. Any required parameters are provided in the yaml file.

## Extending an analyzer

First make a *analyzer_plugins.xml* file, like the following:
```xml
<library path="lib/libstatus_aggregator_analyzers">
  <class name="diagnostic_aggregator/StatusAnalyzer" type="diagnostic_aggregator::StatusAnalyzer" base_class_type="diagnostic_aggregator::Analyzer">
    <description>
      StatusAnalyzer is a diagnostic analyzer for a group of devices (sensors, actuators, power system).
    </description>
  </class>
</library>

```

In the include file (*include/status_aggregator_ros/StatusAnalyzer.h*) you should inherit the base class **AnalyzerGroup** and override the methods **init()** and **report()**: 

```cpp
class StatusAnalyzer : public AnalyzerGroup {
 public:
  StatusAnalyzer();

  bool init(const std::string base_path, const ros::NodeHandle &n) override;
  std::vector<diagnostic_msgs::DiagnosticStatusPtr> report() override;
```

In the cpp file (*src/status_aggregator_ros/StatusAnalyzer.cpp*) change the methods accordingly your needs.

