---
title: DSOR

---

# DSOR

 [More...](#detailed-description)

## Functions

|                | Name           |
| -------------- | -------------- |
| template <typename T \> <br>T | **[saturateControlDepthUsingMinAltitude](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-saturatecontroldepthusingminaltitude)**(T depth_command, T altitude_min, T measured_depth, T measured_altitude)<br>Method to saturate the control in depth.  |
| template <typename T \> <br>T | **[saturateControlAltitudeUsingMinAltitude](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-saturatecontrolaltitudeusingminaltitude)**(T altitude_command, T altitude_min)<br>Method to saturate the control in altitude.  |
| template <typename T \> <br>T | **[saturateControlDepthUsingMaxDepth](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-saturatecontroldepthusingmaxdepth)**(T depth_command, T max_depth)<br>Method to saturate the control in depth.  |
| template <typename T \> <br>T | **[saturateControlAltitudeUsingMaxDepth](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-saturatecontrolaltitudeusingmaxdepth)**(T altitude_command, T measured_depth, T measured_altitude, T max_depth)<br>Method to saturate the control in altitude.  |
| const Eigen::PermutationMatrix< 3 > | **[NED_ENU_REFLECTION_XY](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-ned-enu-reflection-xy)**(Eigen::Vector3i(1, 0, 2) )<br>Use reflections instead of rotations for NED <-> ENU transformation to avoid NaN/Inf floating point pollution across different axes since in NED <-> ENU the axes are perfectly aligned.  |
| template <typename T \> <br>const Eigen::DiagonalMatrix< T, 3 > | **[NED_ENU_REFLECTION_Z](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-ned-enu-reflection-z)**(1 , 1 , - 1) |
| template <typename T \> <br>Eigen::Quaternion< T > | **[rot_body_rotation](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-rot-body-rotation)**(const Eigen::Quaternion< T > & q)<br>Transform a rotation (as a quaternion) from body expressed in ENU (or NED) to inertial frame to a similar rotation (as quaternion) from body expressed in NED (or ENU) to inertial frame.  |
| template <typename T \> <br>Eigen::Quaternion< T > | **[rot_inertial_rotation](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-rot-inertial-rotation)**(const Eigen::Quaternion< T > & q)<br>Transform a rotation (as a quaternion) from body to inertial frame expressed in ENU (or NED) to a similar rotation (as quaternion) from body to inertial frame expressed in NED (or ENU)  |
| template <typename T \> <br>Eigen::Quaternion< T > | **[rot_body_to_inertial](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-rot-body-to-inertial)**(const Eigen::Quaternion< T > & q)<br>Transform a rotation of a rigid body (as a quaternion) from body (ENU or NED) to inertial frame (ENU or NED) to a similar rotation (as quaternion) from body (NED or ENU) to inertial frame (NED or ENU)  |
| template <typename T \> <br>Eigen::Matrix< T, 3, 1 > | **[transform_vect_body_enu_ned](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-transform-vect-body-enu-ned)**(const Eigen::Matrix< T, 3, 1 > & vec)<br>Transform vector in ENU (or NED) to NED (or ENU), expressed in body-frame. +PI rotation around X (Forward) axis transforms from Forward, Right, Down (body frame in NED) Fto Forward, Left, Up (body frame in ENU).  |
| template <typename T \> <br>Eigen::Matrix< T, 3, 1 > | **[transform_vect_between_arbitrary_ref](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-transform-vect-between-arbitrary-ref)**(const Eigen::Matrix< T, 3, 1 > & vec, const Eigen::Quaternion< T > & q)<br>Transform a vector in a given frame of reference to another frame of reference.  |
| template <typename T \> <br>Eigen::Matrix< T, 3, 1 > | **[transform_vect_inertial_enu_ned](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-transform-vect-inertial-enu-ned)**(const Eigen::Matrix< T, 3, 1 > & vec)<br>Transform vector in ENU (or NED) to NED (or ENU), expressed in inertial-frame. ENU <&mdash;> NED - Invert the Z axis and switch the XY axis.  |
| template <typename T \> <br>Eigen::Matrix< T, 3, 3 > | **[transform_cov3_body_enu_ned](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-transform-cov3-body-enu-ned)**(const Eigen::Matrix< T, 3, 3 > & cov_in)<br>Transform 3x3 covariance matrix in ENU (or NED) to NED (or ENU), expressed in body-frame.  |
| template <typename T \> <br>Eigen::Matrix< T, 3, 3 > | **[transform_cov3_inertial_enu_ned](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-transform-cov3-inertial-enu-ned)**(const Eigen::Matrix< T, 3, 3 > & cov_in)<br>Transform 3x3 covariance matrix in ENU (or NED) to NED (or ENU), expressed in inertial-frame.  |
| template <typename T \> <br>int | **[sign](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-sign)**(T v)<br>Returns the sign of the number.  |
| template <typename T \> <br>T | **[saturation](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-saturation)**(T value, T min, T max)<br>A function that saturates 2 values linearly.  |
| template <typename T \> <br>bool | **[approximatelyEquals](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-approximatelyequals)**(T a, T b, T tolerance =1e-6)<br>A function to check if two numbers are equal (int, float, double, etc)  |
| template <typename T \> <br>Eigen::Matrix< T, 3, 1 > | **[quaternion_to_euler](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-quaternion-to-euler)**(const Eigen::Quaternion< T > & q)<br>Function to convert from quaternion to (roll, pitch and yaw), according to Z-Y-X convention This function is from: [https://github.com/mavlink/mavros/issues/444](https://github.com/mavlink/mavros/issues/444) and the logic is also available at: [https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles).  |
| template <typename T \> <br>Eigen::Quaternion< T > | **[euler_to_quaternion](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-euler-to-quaternion)**(const Eigen::Matrix< T, 3, 1 > & v)<br>Converts a vector of euler angles according to Z-Y-X convention into a quaternion.  |
| template <typename T \> <br>T | **[yaw_from_quaternion](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-yaw-from-quaternion)**(const Eigen::Quaternion< T > & q)<br>Gets the yaw angle from a quaternion (assumed a Z-Y-X rotation) NOTE: this function is based on: [https://github.com/mavlink/mavros/blob/ros2/mavros/src/lib/ftf_quaternion_utils.cpp](https://github.com/mavlink/mavros/blob/ros2/mavros/src/lib/ftf_quaternion_utils.cpp) which in turn has the theory explained in: [https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles).  |
| template <typename T \> <br>T | **[wrapTo2pi](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-wrapto2pi)**(T angle)<br>Wrap angle between [0, 2PI].  |
| template <typename T \> <br>T | **[wrapTopi](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-wraptopi)**(T angle)<br>Wrap angle between [-PI, PI].  |
| template <typename T \> <br>T | **[radToDeg](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-radtodeg)**(T angle)<br>Convert an angle in radian to degrees.  |
| template <typename T \> <br>T | **[degToRad](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-degtorad)**(T angle)<br>Convert an angle in degrees to radians.  |
| template <typename T \> <br>T | **[angleDiff](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-anglediff)**(T a, T b)<br>Method to calculate the diference between angles correctly even if they wrap between -pi and pi.  |
| template <typename T \> <br>Eigen::Matrix< T, 3, 3 > | **[computeSkewSymmetric3](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-computeskewsymmetric3)**(const Eigen::Matrix< T, 3, 1 > & v)<br>Compute the 3x3 skew-symmetric matrix from a vector 3x1.  |
| template <typename T \> <br>Eigen::Matrix< T, 2, 2 > | **[computeSkewSymmetric2](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-computeskewsymmetric2)**(T c)<br>Compute the 2x2 skew-symmetric matrix from a constant (int, float or double)  |
| template <typename T \> <br>Eigen::Matrix< T, 3, 3 > | **[rotationAngularBodyToInertial](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-rotationangularbodytoinertial)**(const Eigen::Matrix< T, 3, 1 > & v)<br>Compute the rotation matrix that converts angular velocities expressed in the body frame to angular velocities expressed in the inertial frame (according to Z-Y-X convention) - makes use of small angle approximation.  |
| template <typename T \> <br>Eigen::Matrix< T, 3, 3 > | **[rotationBodyToInertial](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-rotationbodytoinertial)**(const Eigen::Matrix< T, 3, 1 > & v)<br>Method that returns a rotation matrix from body frame to inertial frame, assuming a Z-Y-X convention.  |
| template <typename T \> <br>Eigen::Matrix< T, 3, 1 > | **[spherical_to_cartesian](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#function-spherical-to-cartesian)**(T bearing, T elevation, T range)<br>Convert from spherical to cartesian coordinates. Used mainly with usbl fixes.  |

## Attributes

|                | Name           |
| -------------- | -------------- |
| const Eigen::Quaternion< T > | **[ENU_NED_INERTIAL_Q](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#variable-enu-ned-inertial-q)** <br>Static quaternion to convert a rotation expressed in ENU to a rotation expressed in NED (Z->Y->X convention) on the inertial frame Rotate PI/2 about Z-axis -> Rotate 0 about Y-axis -> Rotate PI about X-axis.  |
| const Eigen::Quaternion< T > | **[ENU_NED_BODY_Q](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#variable-enu-ned-body-q)** <br>Static quaternion to convert a rotation expressed in ENU body frame (ROS base_link) to a rotation expressed in NED body frame (Z->Y->X convention) Rotate 0 about Z-axis -> Rotate 0 about Y-axis -> Rotate PI about X-axis.  |
| const Eigen::Quaternion< T > | **[BODY_ENU_NED_Q](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#variable-body-enu-ned-q)** <br>Static quaternion needed for rotating vectors in body frames between ENU and NED +PI rotation around X (Forward) axis transforms from Forward, Right, Down (body frame in NED) Fto Forward, Left, Up (body frame in ENU).  |
| const Eigen::Transform< T, 3, Eigen::Affine > | **[BODY_ENU_NED_TF](/medusa_base/api/markdown/dsor_utils/dsor_utils/Namespaces/namespaceDSOR/#variable-body-enu-ned-tf)** <br>Static affine matrix to roate vectors ENU (or NED) -> NED (or ENU) expressed in body frame +PI rotation around X (Forward) axis transforms from Forward, Right, Down (body frame in NED) Fto Forward, Left, Up (body frame in ENU).  |

## Detailed Description


Authors: Marcelo Jacinto ([marcelo.jacinto@tecnico.ulisboa.pt](mailto:marcelo.jacinto@tecnico.ulisboa.pt)) Maintained by: Marcelo Fialho Jacinto ([marcelo.jacinto@tecnico.ulisboa.pt](mailto:marcelo.jacinto@tecnico.ulisboa.pt)) Last Update: 10/02/2022 License: MIT File: [vehicle_saturations.hpp] Brief: Defines functions used for saturating the vehicle inputs to the system

Authors: André Potes ([andre.potes@gmail.com](mailto:andre.potes@gmail.com)) Marcelo Jacinto ([marcelo.jacinto@tecnico.ulisboa.pt](mailto:marcelo.jacinto@tecnico.ulisboa.pt)) Maintained by: Marcelo Fialho Jacinto ([marcelo.jacinto@tecnico.ulisboa.pt](mailto:marcelo.jacinto@tecnico.ulisboa.pt)) Last Update: 14/12/2021 License: MIT File: [frames.hpp] Brief: Defines all functions related to conversions between ENU do NED frames and vice-versa

NOTE: Most of this code is adapted from mavros [https://github.com/mavlink/mavros/blob/master/mavros/src/lib/ftf_frame_conversions.cpp](https://github.com/mavlink/mavros/blob/master/mavros/src/lib/ftf_frame_conversions.cpp) which had as authors Nuno Marques ([n.marques21@hotmail.com](mailto:n.marques21@hotmail.com)) and Eddy Scott ([scott.edward@aurora.aero](mailto:scott.edward@aurora.aero))

Authors: Marcelo Jacinto ([marcelo.jacinto@tecnico.ulisboa.pt](mailto:marcelo.jacinto@tecnico.ulisboa.pt)) Joao Quintas ([jquintas@gmail.com](mailto:jquintas@gmail.com)) Joao Cruz ([joao.pedro.cruz@tecnico.ulisboa.pt](mailto:joao.pedro.cruz@tecnico.ulisboa.pt)) Maintained by: Marcelo Fialho Jacinto ([marcelo.jacinto@tecnico.ulisboa.pt](mailto:marcelo.jacinto@tecnico.ulisboa.pt)) Last Update: 14/12/2021 License: MIT File: [math.hpp] Brief: Defines all functions related to general math functions that can be used anywhere

Authors: Marcelo Jacinto ([marcelo.jacinto@tecnico.ulisboa.pt](mailto:marcelo.jacinto@tecnico.ulisboa.pt)) Andre Potes ([andre.potes@tecnico.ulisboa.pt](mailto:andre.potes@tecnico.ulisboa.pt)) Maintained by: Marcelo Fialho Jacinto ([marcelo.jacinto@tecnico.ulisboa.pt](mailto:marcelo.jacinto@tecnico.ulisboa.pt)) Last Update: 14/12/2021 License: MIT File: [rotations.hpp] Brief: Defines all functions related to angle wrapping, rotation matrices, euler angles, convertion to quaternions, etc.

Authors: Joao Quintas ([jquintas@gmail.com](mailto:jquintas@gmail.com)) Joao Cruz ([joao.pedro.cruz@tecnico.ulisboa.pt](mailto:joao.pedro.cruz@tecnico.ulisboa.pt)) Marcelo Jacinto ([marcelo.jacinto@tecnico.ulisboa.pt](mailto:marcelo.jacinto@tecnico.ulisboa.pt)) Maintained by: Marcelo Fialho Jacinto ([marcelo.jacinto@tecnico.ulisboa.pt](mailto:marcelo.jacinto@tecnico.ulisboa.pt)) Last Update: 14/12/2021 License: MIT File: [spherical_coordinates.hpp] Brief: Defines all functions related to spherical coordinates conversions 


## Functions Documentation

### function saturateControlDepthUsingMinAltitude

```cpp
template <typename T >
inline T saturateControlDepthUsingMinAltitude(
    T depth_command,
    T altitude_min,
    T measured_depth,
    T measured_altitude
)
```

Method to saturate the control in depth. 

**Parameters**: 

  * **depth_command** The input command in depth 
  * **altitude_min** The minimum altitude (positive distance from the vehicle to the bottom in m) 
  * **measured_depth** The measured depth of the vehicle 
  * **measured_altitude** The measured altitude of the vehicle 


**Return**: The saturated control of depth such that the vehicle does not surpass the minimum altitude 

### function saturateControlAltitudeUsingMinAltitude

```cpp
template <typename T >
inline T saturateControlAltitudeUsingMinAltitude(
    T altitude_command,
    T altitude_min
)
```

Method to saturate the control in altitude. 

**Parameters**: 

  * **altitude_command** The input command in altitude 
  * **altitude_min** The minimum altitude to keep from the ground 


**Return**: The output command of the altitude saturated such that the vehicle does not surpass the minimum altitude 

### function saturateControlDepthUsingMaxDepth

```cpp
template <typename T >
inline T saturateControlDepthUsingMaxDepth(
    T depth_command,
    T max_depth
)
```

Method to saturate the control in depth. 

**Parameters**: 

  * **depth_command** The input command in depth 
  * **max_depth** The max depth 


**Return**: The output command of the depth saturated such that the vehicle does not surpass the maximum depth 

### function saturateControlAltitudeUsingMaxDepth

```cpp
template <typename T >
inline T saturateControlAltitudeUsingMaxDepth(
    T altitude_command,
    T measured_depth,
    T measured_altitude,
    T max_depth
)
```

Method to saturate the control in altitude. 

**Parameters**: 

  * **altitude_command** The input command in altitude 
  * **measured_depth** The measured depth of the vehicle 
  * **measured_altitude** The measured altitude of the vehicle 
  * **max_depth** The maximum depth allowed 


**Return**: The output command of the altitude saturated such that the vehicle does not surpass the maximum depth 

### function NED_ENU_REFLECTION_XY

```cpp
static const Eigen::PermutationMatrix< 3 > NED_ENU_REFLECTION_XY(
    Eigen::Vector3i(1, 0, 2) 
)
```

Use reflections instead of rotations for NED <-> ENU transformation to avoid NaN/Inf floating point pollution across different axes since in NED <-> ENU the axes are perfectly aligned. 

### function NED_ENU_REFLECTION_Z

```cpp
template <typename T >
static const Eigen::DiagonalMatrix< T, 3 > NED_ENU_REFLECTION_Z(
    1 ,
    1 ,
    - 1
)
```


### function rot_body_rotation

```cpp
template <typename T >
inline Eigen::Quaternion< T > rot_body_rotation(
    const Eigen::Quaternion< T > & q
)
```

Transform a rotation (as a quaternion) from body expressed in ENU (or NED) to inertial frame to a similar rotation (as quaternion) from body expressed in NED (or ENU) to inertial frame. 

**Parameters**: 

  * **q** quaternion representing a rotation: body frame ENU (or NED) -> inertial frame (in arbitrary convention) 


**Return**: quaternion represeting a rotation: body frame NED (or ENU) -> inertial frame (in arbitrary convention) 

NOTE: Check [http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/](http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/) for more details behind these type of transformations towards obtaining rotations in different frames of reference.


### function rot_inertial_rotation

```cpp
template <typename T >
inline Eigen::Quaternion< T > rot_inertial_rotation(
    const Eigen::Quaternion< T > & q
)
```

Transform a rotation (as a quaternion) from body to inertial frame expressed in ENU (or NED) to a similar rotation (as quaternion) from body to inertial frame expressed in NED (or ENU) 

**Parameters**: 

  * **q** quaternion representing a rotation: body frame (in arbitrary convention) -> inertial frame ENU (or NED) 


**Return**: quaternion represeting a rotation: body frame (in arbitrary convention) -> inertial frame NED (or ENU) 

NOTE: Check [http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/](http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/) for more details behind these type of transformations towards obtaining rotations in different frames of reference.


### function rot_body_to_inertial

```cpp
template <typename T >
inline Eigen::Quaternion< T > rot_body_to_inertial(
    const Eigen::Quaternion< T > & q
)
```

Transform a rotation of a rigid body (as a quaternion) from body (ENU or NED) to inertial frame (ENU or NED) to a similar rotation (as quaternion) from body (NED or ENU) to inertial frame (NED or ENU) 

**Parameters**: 

  * **q** quaternion representing a rotation: body frame (ENU or NED) -> inertial frame (ENU or NED) 


**Return**: quaternion representing a rotation: body frame (NED or ENU) -> inertial frame (NED or ENU) 

NOTE: This function is usefull to convert the attitude of a vehicle from "ROS" quaternion to a typicall literature quaternion (where both the body frame and inertial frames are in ENU). If you are converting a quaternion that expresses the orientation of a sensor with respect to a rigid body's body frame (and not the inertial frame), then you DO NOT WANT TO USE THIS FUNCTION. Body-FRAME NED is not the same as INERTIAL-FRAME NED (this comes once again from the fact that in ned body the x-y axis don't switch like in inertial frame) as explained in the documentation.

Essencial only use this if you are representing a body in inertial frame!

NOTE: Check [http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/](http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/) for more details behind these type of transformations towards obtaining rotations in different frames of reference.


### function transform_vect_body_enu_ned

```cpp
template <typename T >
inline Eigen::Matrix< T, 3, 1 > transform_vect_body_enu_ned(
    const Eigen::Matrix< T, 3, 1 > & vec
)
```

Transform vector in ENU (or NED) to NED (or ENU), expressed in body-frame. +PI rotation around X (Forward) axis transforms from Forward, Right, Down (body frame in NED) Fto Forward, Left, Up (body frame in ENU). 

**Parameters**: 

  * **vec** Vector expressed in body-frame (ENU or NED) 


**Return**: Vector expressed in body-frame (NED or ENU) 

### function transform_vect_between_arbitrary_ref

```cpp
template <typename T >
inline Eigen::Matrix< T, 3, 1 > transform_vect_between_arbitrary_ref(
    const Eigen::Matrix< T, 3, 1 > & vec,
    const Eigen::Quaternion< T > & q
)
```

Transform a vector in a given frame of reference to another frame of reference. 

**Parameters**: 

  * **vec** Vector expressed in the original frame of reference 
  * **q** Quaternion that expresses the orientation of the original frame of reference with respect to the final frame of reference 


**Return**: Vector expressed in the new frame of reference 

### function transform_vect_inertial_enu_ned

```cpp
template <typename T >
inline Eigen::Matrix< T, 3, 1 > transform_vect_inertial_enu_ned(
    const Eigen::Matrix< T, 3, 1 > & vec
)
```

Transform vector in ENU (or NED) to NED (or ENU), expressed in inertial-frame. ENU <&mdash;> NED - Invert the Z axis and switch the XY axis. 

**Parameters**: 

  * **vec** Vector expressed in inertial-frame (ENU or NED) 


**Return**: Vector expressed in inertial-frame (NED or ENU) 

### function transform_cov3_body_enu_ned

```cpp
template <typename T >
inline Eigen::Matrix< T, 3, 3 > transform_cov3_body_enu_ned(
    const Eigen::Matrix< T, 3, 3 > & cov_in
)
```

Transform 3x3 covariance matrix in ENU (or NED) to NED (or ENU), expressed in body-frame. 

**Parameters**: 

  * **cov_in** Covariance matrix expressed in body-frame (ENU or NED) 


**Return**: Covariance matrix expressed in body-frame (NED or ENU) 

NOTE: Check [https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance](https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance) for a detailed explanation of the actual conversion proof for covariance matrices


### function transform_cov3_inertial_enu_ned

```cpp
template <typename T >
inline Eigen::Matrix< T, 3, 3 > transform_cov3_inertial_enu_ned(
    const Eigen::Matrix< T, 3, 3 > & cov_in
)
```

Transform 3x3 covariance matrix in ENU (or NED) to NED (or ENU), expressed in inertial-frame. 

**Parameters**: 

  * **cov_in** Covariance matrix expressed in inertial-frame (ENU or NED) 


**Return**: Covariance matrix expressed in inertial-frame (NED or ENU) 

NOTE: Check [https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance](https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance) for a detailed explanation of the actual conversion proof for covariance matrices


### function sign

```cpp
template <typename T >
inline int sign(
    T v
)
```

Returns the sign of the number. 

**Parameters**: 

  * **v** A number 


**Return**: 1 if value is positive 0 if value is 0 -1 if value is negative 

### function saturation

```cpp
template <typename T >
inline T saturation(
    T value,
    T min,
    T max
)
```

A function that saturates 2 values linearly. 

**Parameters**: 

  * **value** A number to saturate 
  * **min** The minimum value 
  * **max** The maximum value 


**Return**: A value such that value in [min, max] 

### function approximatelyEquals

```cpp
template <typename T >
inline bool approximatelyEquals(
    T a,
    T b,
    T tolerance =1e-6
)
```

A function to check if two numbers are equal (int, float, double, etc) 

**Parameters**: 

  * **a** a number to compare 
  * **b** another number to compare 
  * **tolerance** tolerance of comparison 


**Return**: A boolean for true if equal within tolerance, or otherwise 

### function quaternion_to_euler

```cpp
template <typename T >
inline Eigen::Matrix< T, 3, 1 > quaternion_to_euler(
    const Eigen::Quaternion< T > & q
)
```

Function to convert from quaternion to (roll, pitch and yaw), according to Z-Y-X convention This function is from: [https://github.com/mavlink/mavros/issues/444](https://github.com/mavlink/mavros/issues/444) and the logic is also available at: [https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles). 

**Parameters**: 

  * **q** An eigen quaternion 


**Return**: A Vector<T, 3> with the [roll, pitch, yaw] obtained according to Z-Y-X convention 

### function euler_to_quaternion

```cpp
template <typename T >
inline Eigen::Quaternion< T > euler_to_quaternion(
    const Eigen::Matrix< T, 3, 1 > & v
)
```

Converts a vector of euler angles according to Z-Y-X convention into a quaternion. 

**Parameters**: 

  * **v** An eigen vector of either floats or doubles [roll, pitch, yaw] 


**Return**: An Eigen Quaternion 

### function yaw_from_quaternion

```cpp
template <typename T >
inline T yaw_from_quaternion(
    const Eigen::Quaternion< T > & q
)
```

Gets the yaw angle from a quaternion (assumed a Z-Y-X rotation) NOTE: this function is based on: [https://github.com/mavlink/mavros/blob/ros2/mavros/src/lib/ftf_quaternion_utils.cpp](https://github.com/mavlink/mavros/blob/ros2/mavros/src/lib/ftf_quaternion_utils.cpp) which in turn has the theory explained in: [https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles). 

**Parameters**: 

  * **q** A eigen quaternion 


**Return**: The yaw angle in radians (assumed a Z-Y-X rotation) 

### function wrapTo2pi

```cpp
template <typename T >
inline T wrapTo2pi(
    T angle
)
```

Wrap angle between [0, 2PI]. 

**Parameters**: 

  * **angle** angle in radians 


**Return**: The wraped angle 

### function wrapTopi

```cpp
template <typename T >
inline T wrapTopi(
    T angle
)
```

Wrap angle between [-PI, PI]. 

**Parameters**: 

  * **angle** angle in radians 


**Return**: The wraped angle 

### function radToDeg

```cpp
template <typename T >
inline T radToDeg(
    T angle
)
```

Convert an angle in radian to degrees. 

**Parameters**: 

  * **angle** in radians 


**Return**: angle in degrees 

### function degToRad

```cpp
template <typename T >
inline T degToRad(
    T angle
)
```

Convert an angle in degrees to radians. 

**Parameters**: 

  * **angle** in degrees 


**Return**: angle in radians 

### function angleDiff

```cpp
template <typename T >
inline T angleDiff(
    T a,
    T b
)
```

Method to calculate the diference between angles correctly even if they wrap between -pi and pi. 

**Parameters**: 

  * **a** angle 1 in radians 
  * **b** angle 2 in radians 


**Return**: The minimum difference between the two angles 

### function computeSkewSymmetric3

```cpp
template <typename T >
inline Eigen::Matrix< T, 3, 3 > computeSkewSymmetric3(
    const Eigen::Matrix< T, 3, 1 > & v
)
```

Compute the 3x3 skew-symmetric matrix from a vector 3x1. 

**Parameters**: 

  * **v** A vector with 3 elements 


**Return**: A 3x3 skew-symmetric matrix 

### function computeSkewSymmetric2

```cpp
template <typename T >
inline Eigen::Matrix< T, 2, 2 > computeSkewSymmetric2(
    T c
)
```

Compute the 2x2 skew-symmetric matrix from a constant (int, float or double) 

**Parameters**: 

  * **v** A constant 


**Return**: A 2x2 skew-symmetric matrix 

### function rotationAngularBodyToInertial

```cpp
template <typename T >
inline Eigen::Matrix< T, 3, 3 > rotationAngularBodyToInertial(
    const Eigen::Matrix< T, 3, 1 > & v
)
```

Compute the rotation matrix that converts angular velocities expressed in the body frame to angular velocities expressed in the inertial frame (according to Z-Y-X convention) - makes use of small angle approximation. 

**Parameters**: 

  * **v** A vector with 3 elements (roll, pitch, yaw) 


**Return**: A 3x3 rotation matrix 

### function rotationBodyToInertial

```cpp
template <typename T >
inline Eigen::Matrix< T, 3, 3 > rotationBodyToInertial(
    const Eigen::Matrix< T, 3, 1 > & v
)
```

Method that returns a rotation matrix from body frame to inertial frame, assuming a Z-Y-X convention. 

**Parameters**: 

  * **v** A vector with euler angles (roll, pith, yaw) according to Z-Y-X convention 


**Return**: A 3x3 rotation matrix 

### function spherical_to_cartesian

```cpp
template <typename T >
Eigen::Matrix< T, 3, 1 > spherical_to_cartesian(
    T bearing,
    T elevation,
    T range
)
```

Convert from spherical to cartesian coordinates. Used mainly with usbl fixes. 

**Parameters**: 

  * **bearing** Horizontal angle between the direction of an object and another object or between it and the true north direction in degrees. 
  * **elevation** Angle measured between the horizontal and the vehicle line of sight to the object 
  * **range** Distance to the object 
  * **bearing** Horizontal angle between the direction of an object and another object or between it and the true north direction in degrees. 
  * **elevation** Angle measured between the horizontal and the vehicle line of sight to the object 
  * **range** Distance to the object 


**Return**: 

  * Eigen Vector with cartesian coordinates
  * Eigen Vector with cartesian coordinates 



Authors: Joao Quintas ([jquintas@gmail.com](mailto:jquintas@gmail.com)) Joao Cruz ([joao.pedro.cruz@tecnico.ulisboa.pt](mailto:joao.pedro.cruz@tecnico.ulisboa.pt)) Marcelo Jacinto ([marcelo.jacinto@tecnico.ulisboa.pt](mailto:marcelo.jacinto@tecnico.ulisboa.pt)) Maintained by: Marcelo Fialho Jacinto ([marcelo.jacinto@tecnico.ulisboa.pt](mailto:marcelo.jacinto@tecnico.ulisboa.pt)) Last Update: 14/12/2021 License: MIT File: spherical_coordinates.cpp Brief: Defines all functions related to spherical coordinates conversions 



## Attributes Documentation

### variable ENU_NED_INERTIAL_Q

```cpp
static const Eigen::Quaternion< T > ENU_NED_INERTIAL_Q = euler_to_quaternion(Eigen::Matrix<T, 3, 1>(M_PI, 0.0, M_PI_2));
```

Static quaternion to convert a rotation expressed in ENU to a rotation expressed in NED (Z->Y->X convention) on the inertial frame Rotate PI/2 about Z-axis -> Rotate 0 about Y-axis -> Rotate PI about X-axis. 

NOTE: this quaternion is as valid as the quaternion representing the rotation from NED to ENU (quaternion ambiguity) on the inertial frame 


### variable ENU_NED_BODY_Q

```cpp
static const Eigen::Quaternion< T > ENU_NED_BODY_Q = euler_to_quaternion(Eigen::Matrix<T, 3, 1>(M_PI, 0.0, 0.0));
```

Static quaternion to convert a rotation expressed in ENU body frame (ROS base_link) to a rotation expressed in NED body frame (Z->Y->X convention) Rotate 0 about Z-axis -> Rotate 0 about Y-axis -> Rotate PI about X-axis. 

NOTE: this quaternion is as valid as the quaternion representing the rotation from NED to ENU (quaternion ambiguity) on the body frame 


### variable BODY_ENU_NED_Q

```cpp
static const Eigen::Quaternion< T > BODY_ENU_NED_Q = euler_to_quaternion(Eigen::Matrix<T, 3, 1>(M_PI, 0.0, 0.0));
```

Static quaternion needed for rotating vectors in body frames between ENU and NED +PI rotation around X (Forward) axis transforms from Forward, Right, Down (body frame in NED) Fto Forward, Left, Up (body frame in ENU). 

### variable BODY_ENU_NED_TF

```cpp
static const Eigen::Transform< T, 3, Eigen::Affine > BODY_ENU_NED_TF = Eigen::Transform<T, 3, Eigen::Affine>(BODY_ENU_NED_Q<T>);
```

Static affine matrix to roate vectors ENU (or NED) -> NED (or ENU) expressed in body frame +PI rotation around X (Forward) axis transforms from Forward, Right, Down (body frame in NED) Fto Forward, Left, Up (body frame in ENU). 




-------------------------------

Updated on 2022-05-30 at 08:04:26 +0000