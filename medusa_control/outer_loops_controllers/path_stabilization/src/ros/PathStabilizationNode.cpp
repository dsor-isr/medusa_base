#include "PathStabilizationNode.h"

/**
 * @brief  The Path Stabilization Node constructor
 *
 * @param nh  Public ros node handle
 * @param nh_p  Private ros node handle
 */
PathStabilizationNode::PathStabilizationNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p)
    : nh_(*nh), nh_p_(*nh_p) {

    ROS_INFO("in class constructor of PathStabilizationNode");
    this->initializeSubscribers();
    this->initializePublishers();
    /* NOTE: initializeServices is implemented inside PathStabilizationServices.cpp */
    this->initializeServices();
    this->initializeTimer();

    /* Allocate memory for the default Path Stabilization Algorithm - Potes */
    getDefaultControllerPotes();

    /* Set the debugging publisher */
    std::string debug_topic = MedusaGimmicks::getParameters<std::string>(this->nh_p_, "topics/publishers/pstabilization_debug");
    this->ps_algorithm_->setPStabilizationDebugPublisher(this->nh_.advertise<medusa_msgs::mPStabilizationDebug>(debug_topic, 1));
}

/**
 * @brief  Node class destructor
 */
PathStabilizationNode::~PathStabilizationNode() {

    /* Shutdown all the publishers and deleting the memory allocated for the PS
    * controller */
    this->deleteCurrentController();

    /* Shutdown the subscribers common to all controllers */
    this->vehicle_state_sub_.shutdown();
    this->target_state_sub_.shutdown();

    /* Shutdown the publishers common to all controllers */
    this->flag_pub_.shutdown();
    
    /* Stop the timer callback */
    this->timer_.stop();

    /* Shutdown the node */
    this->nh_.shutdown();
}

/**
 * @brief Alocates memory for a default controller. In this case is Potes
 */
void PathStabilizationNode::getDefaultControllerPotes() {
  
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

    /* Get the topic name for the subscribers the are only respective to this algorithm */
    std::string references_topic = MedusaGimmicks::getParameters<std::string>(
        this->nh_p_, "topics/subscribers/references");

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
    this->ps_algorithm_ = new Potes(kp, lambda_p, k_gamma, k_gamma_e, rho_gamma_e,
                    Kp_omega, Q, circle_radius,
                    this->publishers_[0], this->publishers_[1], this->publishers_[2],
                    this->publishers_[3], this->publishers_[4], this->publishers_[5], this->publishers_[6]);

    /* Initialize the subscribers */
    ros::Subscriber references_sub = nh_.subscribe(references_topic, 10, &Potes::updateReferences, (Potes*) this->ps_algorithm_);

    /* Save the special subscribers in the subscriber vector so that they can be eliminated when changing between controllers */
    this->subscribers_.push_back(references_sub);
}

/**
 * @brief  Deletes safely the current path stabilization object
 */
void PathStabilizationNode::deleteCurrentController() {

    /* Delete the path stabilization object */
    if (this->ps_algorithm_) {
        delete this->ps_algorithm_;
        this->ps_algorithm_ = nullptr;
    }

    /* Clear the publishers vector used by the current path stabilization algorithm */
    for (unsigned int i = 0; i < this->publishers_.size(); i++) {
        this->publishers_[i].shutdown();
    }

    /* Clear the subscribers vector used by the current path stabilization algorithm
    */
    for (unsigned int i = 0; i < this->subscribers_.size(); i++) {
        this->subscribers_[i].shutdown();
    }

    /* Empty the list of publishers and subscribers */
    this->publishers_.clear();
    this->subscribers_.clear();
}

/**
 * @brief  Initialize all the subscribers
 */
void PathStabilizationNode::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers for PathStabilizationNode");

    /* Get the topic name for the subscribers */
    std::string vehicle_state_topic = MedusaGimmicks::getParameters<std::string>(
        this->nh_p_, "topics/subscribers/vehicle_state");
    std::string target_state_topic = MedusaGimmicks::getParameters<std::string>(
        this->nh_p_, "topics/subscribers/target_state");
    std::string flag_topic =
        MedusaGimmicks::getParameters<std::string>(this->nh_p_, "flag");

    /* Initialize the subscribers */
    this->vehicle_state_sub_ = nh_.subscribe(vehicle_state_topic, 10, 
                                &PathStabilizationNode::vehicleStateCallback, this);
    this->target_state_sub_ = nh_.subscribe(target_state_topic, 10,
                                &PathStabilizationNode::targetStateCallback, this);
    this->flag_sub_ = nh_.subscribe(flag_topic, 10, 
                                &PathStabilizationNode::flagCallback, this);
}

/**
 * @brief  Method to initialize the publishers of the path stabilization node
 */
void PathStabilizationNode::initializePublishers() {
    ROS_INFO("Initializing Publishers for PathStabilizationNode");

    /* Get the topic name for the publishers */
    std::string flag_topic =
        MedusaGimmicks::getParameters<std::string>(this->nh_p_, "flag");
    
    /* Initialize the publishers */
    this->flag_pub_ = nh_.advertise<std_msgs::Int8>(flag_topic, 1);
}

/**
 * @brief  Path state callback to update the path data
 *
 * @param msg  A pointer to a dsor_paths::PathData that contains information of
 * the path
 */
void PathStabilizationNode::targetStateCallback(const medusa_msgs::mPStabilizationTarget &msg) {

    /* If the algorithm is running, signal that we have received data from the
    * target */
    if (this->timer_.hasStarted()) {
        has_received_target_state = true;
    }

    /* Update the target position and linear velocity (inertial) */
    this->target_state_.target_pos << msg.target_pos.x, msg.target_pos.y, msg.target_pos.z;
    this->target_state_.target_vel << msg.target_vel.x, msg.target_vel.y, msg.target_vel.z;
}

/**
 * @brief  Vehicle state callback to update the state of the vehicle
 *
 * @param msg  A pointer to a auv_msgs::NavigationStatus that contains
 * information regarding the vehicle
 */
void PathStabilizationNode::vehicleStateCallback(
    const auv_msgs::NavigationStatus &msg) {

    /* If the algorithm is running, signal that we have received data from the
    * vehicle state */
    if (this->timer_.hasStarted()) {
        has_received_vehicle_state = true;
    }

    /* Update the vehicle position */
    this->vehicle_state_.eta1 << msg.position.north, msg.position.east,
        msg.position.depth;

    /* Update the vehicle orientation (converting from deg to rad) */
    double roll = MedusaGimmicks::wrap2pi(msg.orientation.x * M_PI / 180, 1);
    double pitch = MedusaGimmicks::wrap2pi(msg.orientation.y * M_PI / 180, 1);
    double yaw = MedusaGimmicks::wrap2pi(msg.orientation.z * M_PI / 180, 1);
    this->vehicle_state_.eta2 << roll, pitch, yaw;

    /* Update the vehicle linear velocity */
    this->vehicle_state_.v1 << msg.body_velocity.x, msg.body_velocity.y,
        msg.body_velocity.z;

    /* Update the vehicle angular velocity (converting from deg to rad) */
    this->vehicle_state_.v2 << msg.orientation_rate.x * M_PI / 180,
        msg.orientation_rate.y * M_PI / 180, msg.orientation_rate.z * M_PI / 180;
}

/**
 * @brief  Initialize the timer callback
 */
void PathStabilizationNode::initializeTimer() {
    this->timer_ =
        nh_.createTimer(ros::Duration(1.0 / PathStabilizationNode::nodeFrequency()),
                        &PathStabilizationNode::timerIterCallback, this);

    /* Wait for the start service to start the Path Stabilization */
    this->timer_.stop();
}

/**
 * @brief  Setup the Node working frequency
 */
double PathStabilizationNode::nodeFrequency() {

    double node_frequency =
        MedusaGimmicks::getParameters<double>(this->nh_p_, "node_frequency", 10);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    return node_frequency;
}

/**
 * @brief Method where the logic is located in order to update the control law
 *
 * @param event  A TimerEvent from ros
 */
void PathStabilizationNode::timerIterCallback(const ros::TimerEvent &event) {

    /* If it has not received data from the path or the vehicle, do not update the
    * algorithm */
    if (!this->has_received_target_state || !this->has_received_vehicle_state)
        return;

    /* Update the values inside the PathStabilization controller */
    this->ps_algorithm_->UpdateVehicleState(this->vehicle_state_);
    this->ps_algorithm_->UpdateTargetState(this->target_state_);

    /* Get the difference between previous update time and current update time */
    ros::Time curr_time = ros::Time::now();
    ros::Duration dt = curr_time - this->prev_time_;
    this->prev_time_ = curr_time;

    /* Compute the control law */
    this->ps_algorithm_->callPSController(double(dt.toSec()));

    /* Publish the results */
    this->ps_algorithm_->publish();

    /* Check if we have reached the end of the path */
    if (this->ps_algorithm_->stop()) {
        /* Ask waypoint algorithm to hold position */
        this->sendWaypoint(WP_FINISH);
        /* Reset the DR postion to the 2d state filter position */
        this->sendResetDeadReckoning();

    }
}

/**
 * @brief  Auxiliar method to send a hold position waypoint
 *
 * @param value -3 to hold the pos after the vehicle, -1 to hold position on the spot
 */
void PathStabilizationNode::sendWaypoint(double value) {
    waypoint::sendWpType1 srv;
    srv.request.x = value;
    srv.request.y = value;
    wp_standard_client_.call(srv);
}

/**
 * @brief  Auxiliar method to reset the DeadReckoning postion  
 * */
void PathStabilizationNode::sendResetDeadReckoning() {
    std_srvs::Trigger srv;
    dr_reset_client_.call(srv);
}

/**
 * @brief  Auxiliar method to stop the algorithm
 */
void PathStabilizationNode::stopAlgorithm() {

    /* Inform the user that path stabilization will stop */
    ROS_INFO("Path Stabilization Node has stop.");

    /* Stop the timer callback where the ps algorithm is invoked */
    this->timer_.stop();

    /* Reset the Path stabilization controller */
    this->ps_algorithm_->reset();

    /* Reset the flags used in the first iteration */
    this->has_received_vehicle_state = false;
    this->has_received_target_state = false;
}

/**
 * @brief  Auxiliar method to stop the algorithm
 */
void PathStabilizationNode::flagCallback(const std_msgs::Int8 &msg) {

    /* If the flag changed and the path stabilization was running, then stop */
    if (msg.data != FLAG_PS && this->timer_.hasStarted()) {
        this->stopAlgorithm();
    }
}

/**
 * @brief  The main function
 *
 * @param argc  The argument count
 * @param argv  The argument array
 */
int main(int argc, char **argv) {

    ros::init(argc, argv, "path_stabilization_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO("main: instantiating an object of type PathStabilizationNode");

    /* Instantiate the PathStabilization Node*/
    PathStabilizationNode PathStabilization(&nh, &nh_p);

    /* Going into spin and let the timer callback do all the work */
    ros::spin();

    return 0;
}
