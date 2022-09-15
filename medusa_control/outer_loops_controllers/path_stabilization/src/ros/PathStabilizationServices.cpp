#include "PathStabilizationNode.h"

/**
 * @brief  A method for initializing all the services. This method is called by
 * the constructor of the PathNode class upon creation
 */
void PathStabilizationNode::initializeServices() {
    ROS_INFO("Initializing Services for PathStabilizationNode");

  /* Get the service names for starting and stoping the path stabilization */
  std::string start_ps_name = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/start_ps", "/start_ps");
  std::string stop_ps_name = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/stop_ps", "/stop_ps");
  std::string update_gains_name = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/updates_gains_ps", "/update_gains_ps");

  /* Get the service names for switching the path stabilization algorithm to use */
  std::string set_potes_name = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/potes_ps", "/set_potes_ps");

  /* Get the service name for reseting the virtual target in the path stabilization */
  std::string reset_virtual_taget_name = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/reset_vt_ps", "/resetVT_ps");
    
  /* Get the service name for waypoint*/
  std::string wp_service_name = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/wp_standard", "/controls/send_wp_standard");

  /* Get the service name for reset_dr*/
  std::string reset_dr_service_name = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/services/reset_dr", "/nav/reset_filter_dr");

  
  /* Advertise the services with these names */
  this->ps_start_srv_ = this->nh_.advertiseService(
      start_ps_name, &PathStabilizationNode::StartPSService, this);
  this->ps_stop_srv_ = this->nh_.advertiseService(
      stop_ps_name, &PathStabilizationNode::StopPSService, this);
  this->ps_update_gains_srv_ = this->nh_.advertiseService(
      update_gains_name, &PathStabilizationNode::UpdateGainsPSService, this);

  this->ps_potes_srv_ = this->nh_.advertiseService(
      set_potes_name, &PathStabilizationNode::SetPotesService, this);

  this->ps_reset_vt_srv_ = this->nh_.advertiseService(
      reset_virtual_taget_name, &PathStabilizationNode::ResetVirtualTargetService, this);
  
  /* Setup the waypoint client needed when mission finishes */
  this->wp_standard_client_ = nh_.serviceClient<waypoint::sendWpType1>(wp_service_name);
  
  /* Setup the reset DeadReckoning client needed when mission finishes */
  this->dr_reset_client_ = nh_.serviceClient<std_srvs::Trigger>(reset_dr_service_name);
  
}

/* Service to start running the path stabilization algorithm that was chosen
 * previously */
bool PathStabilizationNode::StartPSService(path_stabilization::StartPS::Request &req,
    path_stabilization::StartPS::Response &res) {

  /* Check if we have a path stabilization algorithm allocated. If so, start the
  * timer callbacks */
  if (this->ps_algorithm_ != nullptr) { 

    /* Update the last time the iteration of the path stabilization run */
    this->prev_time_ = ros::Time::now();
    this->timer_.start();
    res.success = true;

    /* Publish the code that simbolizes that path stabilization has started */
    MedusaGimmicks::publishValue<std_msgs::Int8, const int>(this->flag_pub_,
        FLAG_PS);

    /* Run the first iteration of the algorithm */
    this->ps_algorithm_->start();

    /* Inform the user the path stabilization algorithm will start */
    ROS_INFO("Path Stabilization is starting.");

    return true;
  }

  /* If there is not object for path stabilization allocated, then print message to
  * console */
  ROS_WARN("There is not path stabilization method allocated. Please restart the "
      "node or set the PS to use.");
  res.success = false;
  return true;
}

/* Service to stop the path stabilization algorithm that was running */
bool PathStabilizationNode::StopPSService(path_stabilization::StopPS::Request &req,
    path_stabilization::StopPS::Response &res) {

  /* Stop the path stabilization only if it was already running */
  if (this->timer_.hasStarted()) {
    /* Publish the code that simbolizes idle mode */
    MedusaGimmicks::publishValue<std_msgs::Int8, const int>(this->flag_pub_,
        FLAG_IDLE);
  }

  /* Return success */
  res.success = true;

  return true;
}

/* Service to reset the virtual target value */
bool PathStabilizationNode::ResetVirtualTargetService(path_stabilization::ResetVT::Request &req,
        path_stabilization::ResetVT::Response &res) {

  /* Check if we have a path stabilization algorithm instantiated */
  if(this->ps_algorithm_ == nullptr) {
    res.success = false;
    return true;
  }

  /* Reset the virtual target */
  res.success = this->ps_algorithm_->resetVirtualTarget((float) req.value);
  return true;
}

/* Servce to update the gains of the path stabilization algorithms live */
bool PathStabilizationNode::UpdateGainsPSService(
    path_stabilization::UpdateGainsPS::Request &req,
    path_stabilization::UpdateGainsPS::Response &res) {

  /* Get the new gains */
  std::vector<double> new_gains = req.gains;

  /* Pass the new gains for the controller */
  if (this->ps_algorithm_ != nullptr) {
    /* Try to update the gains */
    bool result = this->ps_algorithm_->setPSGains(new_gains);

    /* Inform the user if the new gains were accepted or not */
    if (result == true) {
      ROS_INFO("Gains updated successfully!");
    } else {
      ROS_INFO("Gains not accepted!");
    }

    /* Update the response message */
    res.success = result;
    return true;
  }

  /* If the path stabilization algorithm object is not allocated, some error ocurred
   * and we need to restart this node */
  ROS_WARN("There is not path stabilization method allocated. Please restart the "
      "node or set the PS to use.");
  res.success = false;

  return true;
}

/* Service to switch to the Potes Path Stabilization method */
bool PathStabilizationNode::SetPotesService(path_stabilization::SetPS::Request &req,
                                        path_stabilization::SetPS::Response &res) {
                                          
  /* Don't change if the algorithm is running */
  if (this->timer_.hasStarted()) {
    ROS_INFO("Can't change algorithm when PS is running.");
    res.success = false;
    return true;
  }

  /* Clear the memory used by the previous controller */
  this->deleteCurrentController();

  /* Get the topic names for the publishers */
  std::string surge_topic = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/surge");
  std::string sway_topic = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/sway");
  std::string heave_topic = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/heave");

  std::string roll_rate_topic = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/roll_rate");
  std::string pitch_rate_topic = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/pitch_rate");
  std::string yaw_rate_topic = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/yaw_rate");

  std::string rabbit_topic = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/rabbit");

  std::string pstabilization_debug_topic = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/publishers/pstabilization_debug");

  /* Get the topic name for the subscribers the are only respective to this algorithm */
  std::string references_topic = MedusaGimmicks::getParameters<std::string>(
      this->nh_p_, "topics/subscribers/path_stabilization/references");

  /* Create the publishers for the node */
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(surge_topic, 1));
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(sway_topic, 1));
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(heave_topic, 1));

  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(roll_rate_topic, 1));
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(pitch_rate_topic, 1));
  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(yaw_rate_topic, 1));

  this->publishers_.push_back(nh_.advertise<std_msgs::Float64>(rabbit_topic, 1));

  /* Variables to store the gains of the controller */
  double kp, lambda_p, k_gamma, k_gamma_e, rho_gamma_e, circle_radius;
  std::vector<double> Kp_omega_vector;
  std::vector<double> Q_vector;

  try {

    /* Read the gains for the controller */
    nh_p_.getParam("controller_gains/potes/kp", kp);
    nh_p_.getParam("controller_gains/potes/lambda_p", lambda_p);
    nh_p_.getParam("controller_gains/potes/k_gamma", k_gamma);
    nh_p_.getParam("controller_gains/potes/k_gamma_e", k_gamma_e);
    nh_p_.getParam("controller_gains/potes/rho_gamma_e", rho_gamma_e);
    nh_p_.getParam("controller_gains/potes/Kp_omega", Kp_omega_vector);
    nh_p_.getParam("controller_gains/potes/Q", Q_vector);
    nh_p_.getParam("controller_gains/potes/circle_radius", circle_radius);

    Eigen::Vector3d Kp_omega(Kp_omega_vector.data());
    Eigen::Matrix3d Q(Q_vector.data());

    /* Assign the new controller */
    this->ps_algorithm_
        = new Potes(kp, lambda_p, k_gamma, k_gamma_e, rho_gamma_e,
                    Kp_omega, Q, circle_radius,
                    this->publishers_[0], this->publishers_[1], this->publishers_[2],
                    this->publishers_[3], this->publishers_[4], this->publishers_[5], this->publishers_[6]);

    /* Path the debug variables publisher to the class*/
    ps_algorithm_->setPStabilizationDebugPublisher(nh_p_.advertise<medusa_msgs::mPStabilizationDebug>(pstabilization_debug_topic,1));

    res.success = true;

    /* Initialize the subscribers */
    ros::Subscriber references_sub = nh_.subscribe(references_topic, 10, &Potes::updateReferences, (Potes*) this->ps_algorithm_);

    /* Save the special subscribers in the subscriber vector so that they can be eliminated when changing between controllers */
    this->subscribers_.push_back(references_sub);

  } catch (...) {
    ROS_WARN("Some error occured. Please reset the PS node for safety");
    res.success = false;
    return false;
  }

  /* Return success */
  ROS_INFO("PS controller switched to Potes. This is not a true path controller, but rather a path positioning/stabilization controller.");
  return true;
}
