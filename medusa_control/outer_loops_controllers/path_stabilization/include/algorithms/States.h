#pragma once

#include <Eigen/Core>
#include <limits>

/**
 * @brief A structure to hold the state of the vehicle
 * @author    André Potes
 * @version   1.0a
 * @date      2021
 * @copyright MIT
 */
typedef struct {

  /**
   * @brief Velocities in the body frame
   */
  Eigen::Vector3d v1{0.0, 0.0, 0.0};    // Linear velocities in the body frame
  Eigen::Vector3d v2{0.0, 0.0, 0.0};    // Angular velocities in the body frame

  /**
   * @brief Positions and orientations in inertial frame
   */
  Eigen::Vector3d eta1{0.0, 0.0, 0.0};  // Position relative to the inertial frame
  Eigen::Vector3d eta2{0.0, 0.0, 0.0} ; // Orientation relative to the inertial frame

 } VehicleState;


/**
 * @brief  A structure to hold the data of the path
 * @author    André Potes
 * @version   1.0a
 * @date      2021
 * @copyright MIT
 */
typedef struct {

  /**
   * @brief Target position and velocity (both inertial)
   */
  Eigen::Vector3d target_pos{0.0, 0.0, 0.0};
  Eigen::Vector3d target_vel{0.0, 0.0, 0.0};

} TargetState;

typedef struct {

  std::string algorithm;
  double cross_track_error{0.0};
  double along_track_error{0.0};
  double vertical_track_error{0.0};
  double position_error_norm{0.0};
  double theta_e{0.0};
  double gamma_e{0.0};

} PStabilizationDebug;

