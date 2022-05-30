---
title: Classes

---

# Classes




* **struct [Output_t](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/structOutput__t/)** <br>Waypoint output struct. Contains orientation and linear and angular velocities. Not everything needs to be used. 
* **struct [Vehicle_t](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/structVehicle__t/)** <br>Vehicle state struct. Contains position, orientation and linear and angular velocities. 
* **struct [WPref_t](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/structWPref__t/)** <br>Waypoint reference struct. Contains references for poistion and orientations. Not everything needs to be used. 
* **class [WaypointController](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointController/)** <br>Abstract class of a waypoint controller. 
* **class [WaypointNode](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWaypointNode/)** <br>ROS node class. 
* **class [WpHeading](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpHeading/)** <br>Waypoint controller using surge, surge and yaw rate. Not only can go to waypoint and hold its position but can also maintain heading. 
* **class [WpLoose](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpLoose/)** <br>Waypoint controller similar to standard, using surge and yaw (nose of the vehicle points to the desired position). The difference is that this one limits the rate of yaw reference. 
* **class [WpStandard](/medusa_base/api/markdown/medusa_control/outer_loops_controllers/waypoint/Classes/classWpStandard/)** <br>Waypoint controller using surge and yaw, where the nose of the vehicle points to the desired position. 



-------------------------------

Updated on 2022-05-30 at 08:04:30 +0000
