#include "PathStabilization.h"

/* Virtual destructor for the path stabilization class */
PathStabilization::~PathStabilization() {}

/* Setter for the vehicle state structure */
void PathStabilization::UpdateVehicleState(const VehicleState &vehicle_state) {
  this->vehicle_state_ = vehicle_state;
}

/* Setter for the path state structure */
void PathStabilization::UpdateTargetState(const TargetState &target_state) {
  this->target_state_ = target_state;
}

void PathStabilization::publish() {
  
  publish_private();

  medusa_msgs::mPStabilizationDebug ps_debug;

  ps_debug.header.stamp = ros::Time::now();
  ps_debug.algorithm = pstabilization_debug_.algorithm;
  ps_debug.cross_track_error = pstabilization_debug_.cross_track_error;
  ps_debug.along_track_error = pstabilization_debug_.along_track_error;
  ps_debug.vertical_track_error = pstabilization_debug_.vertical_track_error;
  ps_debug.position_error_norm = pstabilization_debug_.position_error_norm;
  ps_debug.theta_e = pstabilization_debug_.theta_e;
  ps_debug.gamma_e = pstabilization_debug_.gamma_e;
  
  pstabilization_debug_pub_.publish(ps_debug);
}

/* Auxiliar method to smooth out the angle to be used by path stabilization algorithms */
double PathStabilization::algConvert(double alg_new, double alg_old, double alg_out_old) {
  double alg_e, alg_out;
  alg_e = alg_new - alg_old;
  if (alg_e > 3 * MedusaGimmicks::PI / 2) {
    alg_out = alg_out_old - 2 * MedusaGimmicks::PI + alg_e;
  } else if (alg_e < -3 * MedusaGimmicks::PI / 2) {
    alg_out = alg_out_old + 2 * MedusaGimmicks::PI + alg_e;
  } else {
    alg_out = alg_out_old + alg_e;
  }
  return alg_out;
}

/* Method to reset the virtual target of the vehicle (gamma) to a pre-specified value. */
bool PathStabilization::resetVirtualTarget(float value) {

  /* By default do nothing, unless the controller implements this method */
  return true;
}

/* Method to reset the virtual target of the vehicle (gamma) to zero */
bool PathStabilization::resetVirtualTarget() {
  return this->resetVirtualTarget(0.0);
}