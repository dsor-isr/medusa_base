# Updater example

To use ROS diagnostics add the following to CMakeLists.txt:
```
find_package(catkin REQUIRED
 COMPONENTS
 std_msgs 
 medusa_msgs
 roscpp
 **diagnostic_msgs**
 serial_lib
 nmea_msgs
 medusa_gimmicks_library
 **medusa_diagnostics_library**
)
```
and in package.xml guarantee the following two dependencies:
```xml
<depend>diagnostic_msgs</depend>
<depend>medusa_diagnostics_library</depend>
```
## Code use

In some header file please include the following:

```cpp
#include <diagnostic_msgs/KeyValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <medusa_diagnostics_library/MedusaDiagnostics.h>
```

Below you can find an example, with two different key values(pressure and temperature), the status message is directly populated in the driver node:

```cpp
void ImsNode::reportingPressure(const medusa_msgs::Pressure &msg)
{
	//** Instantiate diagnostic message
  	diagnostic_msgs::DiagnosticArray diag_msg;
 	diag_msg.header.stamp = msg.header.stamp;
  	diag_msg.status.push_back(MedusaDiagnostics::setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus::OK, 
                            "/Sensors/Pressure", "IMS Pressure Good.", "IMS Board"));

	diag_msg.status.push_back(MedusaDiagnostics::setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus::OK, 
                            "/Sensors/Temperature", "IMS Temperature Good.", "IMS Board"));


	// +.+ Check pressure bounds
	if (MedusaDiagnostics::checkLowerBound(msg.pressure, p_min_pressure_) || MedusaDiagnostics::checkUpperBound(msg.pressure, p_max_pressure_))
		MedusaDiagnostics::errorLevel(&diag_msg, ros::this_node::getName() + ": Pressure out of bounds", 0);
	
	// +.+ Check temperature bounds
	if (MedusaDiagnostics::checkLowerBound(msg.temperature, p_min_temp_) || MedusaDiagnostics::checkUpperBound(msg.temperature, p_max_temp_))
		MedusaDiagnostics::errorLevel(&diag_msg, ros::this_node::getName() + ": Temperature out of bounds", 1);
	
	// +.+ Pressure and temperature key values 
  	MedusaDiagnostics::addKeyValue(&diag_msg, "Pressure", boost::str(boost::format("%.0f") % (msg.pressure)), 0);
  	MedusaDiagnostics::addKeyValue(&diag_msg, "Temperature", boost::str(boost::format("%.0f") % (msg.temperature)), 1);
  
  	diagnostics_pub_.publish(diag_msg);
}
```

where:

```cpp
diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100);
```