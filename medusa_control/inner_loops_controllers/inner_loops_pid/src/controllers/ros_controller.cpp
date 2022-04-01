#include "ros_controller.h"

RosController::RosController(ros::NodeHandle &nh, std::string controller_name,
                             std::string refCallback_topic, double *state,
                             double *force_or_torque)
    : state_ptr_(state), controller_name_(controller_name),
      force_or_torque_ptr_(force_or_torque) {
  init(nh, controller_name, refCallback_topic);
}

void RosController::init(ros::NodeHandle &nh, std::string controller_name,
                         std::string refCallback_topic) {
  // default state not angle units
  setCircularUnits(false);
  // positive output sign
  setPositiveOutput(true);

  // read parameters
  double timout = nh.param("timout_ref", 1.5);
  double kp = nh.param("controllers/" + controller_name + "/kp", 0.0);
  double ki = nh.param("controllers/" + controller_name + "/ki", 0.0);
  double kd = nh.param("controllers/" + controller_name + "/kd", 0.0);
  double kff = nh.param("controllers/" + controller_name + "/kff", 0.0);
  double kff_d = nh.param("controllers/" + controller_name + "/kff_d", 0.0);
  double kff_dd = nh.param("controllers/" + controller_name + "/kff_dd", 0.0);
  double max_out = nh.param("controllers/" + controller_name + "/max_out", 0.0);
  double min_out = nh.param("controllers/" + controller_name + "/min_out", -max_out);
  double max_error = nh.param("controllers/" + controller_name + "/max_err", 0.0);
  double min_error = nh.param("controllers/" + controller_name + "/min_err", -max_error);
  max_ref_value_ = nh.param("controllers/" + controller_name + "/max_ref", 0.0);
  min_ref_value_ = nh.param("controllers/" + controller_name + "/min_ref", 0.0);

  // Don't create the controller if no gains were specified
  if (kp == 0.0 && ki == 0.0 && kd == 0.0 && kff == 0.0 && kff_d == 0.0 && kff_dd == 0.0) {
    pid_c_ = NULL;
    return;
  }

  // subscribe to relevant topic
  ros_sub_ = nh.subscribe(refCallback_topic.c_str(), 10,
                          &RosController::refCallback, this);

  // create the PID controller
  pid_c_ = new PID_Controller(kp, ki, kd, kff, kff_d, kff_dd, max_error, max_out, min_error, min_out);

  // initialize variables
  ref_value_ = 0.0;
  ref_time_ = ros::Time(0.0);
  timeout_ref_ = ros::Duration(timout);
  last_cmd_ = ros::Time::now();
}

void RosController::refCallback(const std_msgs::Float64 &msg) {
  ref_time_ = ros::Time::now();
  if (max_ref_value_ == 0.0 && min_ref_value_ == 0)
    ref_value_ = msg.data;
  else
    // saturate references
    ref_value_ = fmin(fmax(msg.data, min_ref_value_), max_ref_value_);
}

double RosController::computeCommand() {

  if (!validRef())
    return 0.0;

  double error = ref_value_ - *state_ptr_;
  if (isnan(ref_value_)) {
    ROS_ERROR("getting NaN in %s controller", controller_name_.c_str());
    return 0.0;
  }

  // Wrap the error between [-180,180]
  if (circular_units_) {
    if (error > 180)
      error -= 360;

    if (error < -180)
      error += 360;
  }

  ros::Time tnow = ros::Time::now();

  // Call the controller
  *force_or_torque_ptr_ += (positive_output_ ? 1 : -1) * pid_c_->computeCommand(error, ref_value_, (tnow - last_cmd_).toSec());
  last_cmd_ = tnow;

  return *force_or_torque_ptr_;
}

bool RosController::validRef() {
  if (pid_c_ == NULL)
    return false;

  if ((ros::Time::now() - ref_time_) < timeout_ref_) {
    // reactivate the controller if needed
    if (pid_c_->disable) {
      last_cmd_ = ros::Time::now();
      pid_c_->reset();
      pid_c_->disable = false;
    }
    return true;
  }
  pid_c_->disable = true; // disable controller
  return false;
}
