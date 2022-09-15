#include "Potes.h"

/* The constructor for the Potes Path Stabilization Controller */
Potes::Potes(double kp, double lambda_p, double k_gamma, double k_gamma_e, double rho_gamma_e,
             Eigen::Vector3d &Kp_omega,  Eigen::Matrix3d &Q, double circle_radius,
             ros::Publisher surge_pub, ros::Publisher sway_pub, ros::Publisher heave_pub,
             ros::Publisher roll_rate_pub, ros::Publisher pitch_rate_pub, ros::Publisher yaw_rate_pub,
             ros::Publisher rabbit_pub) {

    /* Initiate the gains of the controller */
    this->kp_ = kp;
    this->lambda_p_ = lambda_p;
    this->k_gamma_ = k_gamma_;
    this->k_gamma_e_ = k_gamma_e;
    this->rho_gamma_e_ = rho_gamma_e_;
    this->Kp_omega_ = Kp_omega;

    /* Safety radius for the vehicle path */
    this->circle_radius_ = circle_radius;

    /* Save the publishers, to latter publish the data */
    this->surge_pub_ = surge_pub;
    this->sway_pub_ = sway_pub;
    this->heave_pub_ = heave_pub;

    this->roll_rate_pub_ = roll_rate_pub;
    this->pitch_rate_pub_ = pitch_rate_pub;
    this->yaw_rate_pub_ = yaw_rate_pub;

    this->rabbit_pub_ = rabbit_pub;
}

/* Method used to set the path stabilization gains */
bool Potes::setPSGains(std::vector<double> gains) {

    /* Handle the case where the number of gains received does not coincide witht the
     * number of gains of the algorithm
     */
    if(gains.size() != 8) {
        return false;
    }

    /* Check that all gains are positive */
    for(int i = 0; i < 8; i++) {
        if(gains[i] < 0) return false;
    }

    /* Initiate the gains of the controller */
    this->kp_ = gains[0]; 
    this->lambda_p_ = gains[1];
    this->k_gamma_ = gains[2]; 
    this->k_gamma_e_ = gains[3]; 
    this->rho_gamma_e_ = gains[4]; // maximum distance to virtual target (compromise between path tracking and target stabilization)
    this->Kp_omega_ << gains[5], gains[6], gains[7];

    return true; //gains where updated successfully
}

/* Method to publish_private the data computed from the algorithm */
void Potes::publish_private() {

    /* Publish the control references for linear velocities */
    MedusaGimmicks::publishValue<std_msgs::Float64, const double>(this->surge_pub_, this->desired_surge_);
    MedusaGimmicks::publishValue<std_msgs::Float64, const double>(this->sway_pub_, this->desired_sway_);
    MedusaGimmicks::publishValue<std_msgs::Float64, const double>(this->heave_pub_, this->desired_heave_);

    /* Publish the control references for angular velocities */
    //MedusaGimmicks::publishValue<std_msgs::Float64, const double>(this->roll_rate_pub_, this->desired_roll_rate_);
    //MedusaGimmicks::publishValue<std_msgs::Float64, const double>(this->pitch_rate_pub_, this->desired_pitch_rate_);
    MedusaGimmicks::publishValue<std_msgs::Float64, const double>(this->yaw_rate_pub_, this->desired_yaw_rate_);

    /* Publish the virtual targets value */
    MedusaGimmicks::publishValue<std_msgs::Float64, const double>(this->rabbit_pub_, this->gamma_);
}

/* Method that will run in the first iteration of the algorithm */
void Potes::start() {

    /* Publish the initial virtual target value to get the data from the path */
    MedusaGimmicks::publishValue<std_msgs::Float64, const double>(this->rabbit_pub_, this->gamma_);
}

/* Method used to check whether we reached the end of the algorithm or not */
bool Potes::stop() {

  /**
   * Check if the gamma is greater then the gamma max of the path
   * If so, we have reached the end
   */
  // if((target_state_.target_pos - this->mission_pos_).norm() <= 1.0) return true;

  return false;
}

/* Method to reset all the algorithm data when the path stabilization restarts */
bool Potes::reset() {

  /* Reset the velocity references */
  this->desired_surge_ = 0.0;
  this->desired_sway_ = 0.0;
  this->desired_heave_ = 0.0;
  this->desired_roll_rate_ = 0.0;
  this->desired_pitch_rate_ = 0.0;
  this->desired_yaw_rate_ = 0.0;

  /* Reset the virtual target */
  this->gamma_ref_dot_ = 0.0;
  this->gamma_ref_= 0.0;
  this->gamma_dot_= 0.0;
  this->gamma_= 0.0;

  /* Reset the mission and target boolean variables */
  this->has_received_references_ = false;

  return true;
}

/* Method to reset the virtual target of the vehicle (gamma) */
bool Potes::resetVirtualTarget(float value) {

    /* Reset the virtual target */

    // IGNORE AS THIS DOES NOT MAKE SENSE IN THIS APPLICATION

    return true;
}


/* Method that updates the mission position */
void Potes::updateReferences(const medusa_msgs::mPStabilizationRefs &msg) {

    this->has_received_references_ = true;
    
    this->mission_pos_ << msg.mission_pos.x, msg.mission_pos.y, msg.mission_pos.z;
    this->desired_attitude_  << msg.desired_roll, msg.desired_pitch, msg.desired_heading;
}

/* Method that implements the control law */
void Potes::callPSController(double dt) {

    /* Check if we already have the necessary references from the guidance system */
    if(!this->has_received_references_) return;
    
    /* Inertial irrotational ocean currents - TODO */
    Eigen::Vector3d Vc_v = Eigen::MatrixXd::Zero(3,1);

    /* Compute the vehicle rotation matrix from vehicle orientation state in Euler Angles (in radians)  */
    Eigen::Matrix3d Rv = DSOR::rotationBodyToInertial<double>(this->vehicle_state_.eta2);

    /* Get the necessary references for the path stabilization controller:
     * desired position: pd
     * reference gamma: gamma_ref
     * derivative of reference gamma: gamma_ref_dot
     * desired rotation matrix: Rd */
    /* -------------------------------------------------------------------------------------------------- */
    std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Vector3d, Eigen::Vector3d> reference_signals;
    Eigen::Vector3d pd, pd_t_gamma, dpd_t_dgamma; Eigen::Matrix3d Rd, Rt, St;

    std::tie(pd, Rd, Rt, St, pd_t_gamma, dpd_t_dgamma) = this->getReferences();
    
    /* Get the vehicle position in the 3D plane */
    Eigen::Vector3d p; p = vehicle_state_.eta1; // using depth instead of altitude

    Eigen::Vector3d ep = Rv.transpose() * (p - pd);               // compute the position error in the body frame
    double gamma_e = this->gamma_ - this->gamma_ref_; // compute the gamma error
    Eigen::Matrix3d Re; Re = Rv.transpose() * Rd;           // compute the rotation matrix error

    /* ----------------------------- Compute the linear velocity control law  -------------------------- */
    /* -------------------------------------------------------------------------------------------------- */

    Eigen::Vector3d vd, x;
    x = (this->kp_ / this->lambda_p_)*ep;
    vd = -this->lambda_p_ * this->sigma(x)
         -Rv.transpose() * (Vc_v - target_state_.target_vel - Rt * (dpd_t_dgamma * this->gamma_ref_dot_ + St * pd_t_gamma));
    
    /* Save the control variables to be published later */
    this->desired_surge_ = vd(0);
    this->desired_sway_ = vd(1);
    this->desired_heave_ = vd(2);

    /* ----------------------------- Compute the virtual particle control law  -------------------------- */
    /* -------------------------------------------------------------------------------------------------- */
    this->gamma_dot_ = this->gamma_ref_dot_ -
                       this->k_gamma_ * (rho_gamma_e_ * std::tanh(this->k_gamma_e_ * gamma_e) - ep.transpose() * Rv.transpose() * Rt * dpd_t_dgamma);

    /* Integrate to get the virtual target position */
    this->gamma_ += this->gamma_dot_ * dt;

    /* ----------------------------- Compute the angular velocity control law  -------------------------- */
    /* -------------------------------------------------------------------------------------------------- */
    Eigen::Vector3d wd; Eigen::Matrix3d M;
    M = Re*this->Q_*this->Q_.transpose() - this->Q_*this->Q_.transpose()*Re.transpose();

    Eigen::Vector3d X;
    X << M(2,1), M(0,2), M(1,0);
    wd = this->Kp_omega_.asDiagonal() * X;
    /* Save the angular control variables to be published later */
    this->desired_roll_rate_ = wd(0);
    this->desired_pitch_rate_ = wd(1);
    this->desired_yaw_rate_ = wd(2);

    /* Path stabilization values for debug */
    pstabilization_debug_.algorithm = "Potes";
    pstabilization_debug_.along_track_error = ep[0];
    pstabilization_debug_.cross_track_error = ep[1];
    pstabilization_debug_.vertical_track_error = ep[2];
    pstabilization_debug_.position_error_norm = ep.norm();

    Eigen::AngleAxisd ReAngleAxis(Re); // get angle-axis representation of rotation matrix
    pstabilization_debug_.theta_e = ReAngleAxis.angle();
    pstabilization_debug_.gamma_e = gamma_ - gamma_ref_;
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Vector3d, Eigen::Vector3d> Potes::getReferences() {

    /* ---------------- Compute the reference signals for the path stabilization controller ----------------- */
    /* -------------------------------------------------------------------------------------------------- */

    Eigen::Vector3d p; p = vehicle_state_.eta1;
    Eigen::Vector3d pd_t_gamma;
    Eigen::Vector3d dpd_t_dgamma;

    Eigen::MatrixXd rot_factor(3,2);        // rotation conversion to make a vertical circle from the horizontal circle
    Eigen::Vector2d dist2d_target_mission;  // distance to the mission position
    Eigen::Vector3d i_hat;                  // orthogonal mission bearing unit row vector
    Eigen::Vector3d j_hat;                  // orthogonal vertical unit row vector

    dist2d_target_mission = this->mission_pos_.head(2) - target_state_.target_pos.head(2);
    if (dist2d_target_mission.isZero(0)) {
        // target mission is the mission position, arbitrate circle face north
        i_hat <<  1, 0, 0;
    }
    else {
        // else make circle face mission location
        Eigen::Vector2d aux; aux = dist2d_target_mission/dist2d_target_mission.norm();
        i_hat <<  aux(0), aux(1), 0;
    }
    j_hat << 0, 0, 1;
    rot_factor << i_hat, j_hat;

    Eigen::Vector2d p_gamma_circle; p_gamma_circle << std::cos(gamma_), std::sin(gamma_);
    pd_t_gamma = this->circle_radius_ * rot_factor * p_gamma_circle;
    Eigen::Vector2d pd_gamma_circle; pd_gamma_circle << -std::sin(gamma_), std::cos(gamma_);
    dpd_t_dgamma = this->circle_radius_ * rot_factor * pd_gamma_circle;
    double chi = std::atan2(this->mission_pos_(1) - target_state_.target_pos(1),
                            this->mission_pos_(0) - target_state_.target_pos(0));     // Bearing between diver and mission site
    double chi_dot, X, X_dot;
    if(this->mission_pos_(0) != target_state_.target_pos(0)) {
        X = this->mission_pos_(1) - target_state_.target_pos(1) / this->mission_pos_(0) - target_state_.target_pos(0);
        X_dot = (-(this->mission_pos_(1) - target_state_.target_pos(1)) * target_state_.target_vel(0) -(this->mission_pos_(0) - target_state_.target_pos(0)) * target_state_.target_vel(1)) /
                std::pow(this->mission_pos_(0) - target_state_.target_pos(0), 2);
    } else {
        X = M_PI / 2;
        X_dot = 0.0;
    }
    chi_dot = X_dot / (1 + std::pow(X, 2));

    Eigen::Vector3d wt = Eigen::MatrixXd::Zero(3,1); wt(2) = chi_dot; // angular velocity of target frame
    Eigen::Matrix3d St = DSOR::computeSkewSymmetric3<double>(wt); // skew symmetric for later computation of control law
    Eigen::Vector3d Theta_t; Theta_t << 0.0, 0.0, chi;
    Eigen::Matrix3d Rt = DSOR::rotationBodyToInertial<double>(Theta_t); // rotation Matrix from target frame to inertial frame
    
    Eigen::Vector3d pd; pd = Rt * pd_t_gamma + target_state_.target_pos; // center path in target position

    /* Compute the desired parameterization variable and respective derivative */
    double d = (this->mission_pos_.head(2) - target_state_.target_pos.head(2)).norm();
    double d_dot = (-(this->mission_pos_(1) - target_state_.target_pos(1)) * target_state_.target_vel(0) - (this->mission_pos_(0) - target_state_.target_pos(0)) * target_state_.target_vel(1)) / d;
    this->gamma_ref_ = std::atan2(this->mission_pos_(2) - target_state_.target_pos(2), d);

    double Y = this->mission_pos_(2) - target_state_.target_pos(2) / d;
    double Y_dot = -(target_state_.target_vel(2) * d + (this->mission_pos_(2) - target_state_.target_vel(2)) * d_dot) / std::pow(d, 2);
    this->gamma_ref_dot_ = Y_dot / (1+std::pow(Y, 2));

    /* ------------------- Compute the reference signals for the attitude controller -------------------- */
    /* -------------------------------------------------------------------------------------------------- */

    /* Compute desired rotation matrix */
    Eigen::Matrix3d Rd;
    Rd = DSOR::rotationBodyToInertial<double>(this->desired_attitude_);

    return std::make_tuple(pd, Rd, Rt, St, pd_t_gamma, dpd_t_dgamma);
}

/* Method that computes a saturation function */
Eigen::Vector3d Potes::sigma(Eigen::Vector3d &s) {

    Eigen::Vector3d sigma_;

    for(int i=0; i < 3; i++)
        sigma_(i) = (std::abs(s(i)) >=1) ? std::copysign(1.0, s(i)) : 1.0 * s(i);

    return sigma_;
}