---
title: Classes

---

# Classes




* **class [DeadReckoning](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classDeadReckoning/)** <br>[DeadReckoning]() class. 
* **namespace [FilterGimmicks](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Namespaces/namespaceFilterGimmicks/)** <br>[FilterGimmicks]() namespace. 
    * **struct [measurement](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1measurement/)** <br>Define a measurement object. 
    * **struct [predicate_frame_id](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structFilterGimmicks_1_1predicate__frame__id/)** <br>Predicate to find a giver frame_id among a list of frame_id. 
* **class [FiltersNode](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classFiltersNode/)** <br>[FiltersNode]() class. 
* **class [HorizontalFilter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classHorizontalFilter/)** <br>Horizontal Filter class. 
    * **struct [config](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structHorizontalFilter_1_1config/)** 
* **class [RotationalFilter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classRotationalFilter/)** <br>This Class estimates the state of the vehicle in the rotational frame. The state includes orientation and angular velocity. The latest state estimate of the filter is obtained using the function [computePredict()](). Measurement updates to the filter is done using [newMeasurement()](). 
    * **struct [config](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structRotationalFilter_1_1config/)** 
* **class [VerticalFilter](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/classVerticalFilter/)** <br>This Class estimates the state of the vehicle in the horizontal frame. The state includes depth, velocity, altitude, and bouyancy. The latest state estimate of the filter is obtained using the exposed function getEstimate(). Measurement updates to the filter is done either using measCallback(). 
    * **struct [config](/medusa_base/api/markdown/medusa_nav/sensor_fusion/Classes/structVerticalFilter_1_1config/)** 



-------------------------------

Updated on 2022-05-30 at 08:04:29 +0000
